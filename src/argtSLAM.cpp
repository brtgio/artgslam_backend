#include "argtSlam.hpp"

// Variables globales
double yaw = 0.0;
float xx = 0.0f, yy = 0.0f;

// Variables para el filtro FIR (orden 40)
static float samplesSonar_x[8][41];//(41, 0.0f);
static float samplesSonar_y[8][41];//(41, 0.0f);

// Inicialización de variables estáticas de clase
double argtSLAM::theta_ant = 0.0;
double argtSLAM::theta_act = 0.0;
int argtSLAM::numRotation = 0;
const double argtSLAM::corFactor = 1.025;
std_msgs::Float32 yawAmigobot;

  ros::Publisher yawPub;

// Constructor
argtSLAM::argtSLAM()
    : linear_(1), angular_(0), l_scale_(0.5), a_scale_(0.5)
{
    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);

    sub_odom = nh_.subscribe("/RosAria/pose", 1000, odomchatterCallback);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);

    sonarPoincloud_filter = nh_.advertise<sensor_msgs::PointCloud>("sonarPublisher_batman_filter", 1000);
    sonarPoincloudraw = nh_.advertise<sensor_msgs::PointCloud>("sonarPublisher_brunodias_raw", 1000);
    sonarFilterdata_bag = nh_.advertise<geometry_msgs::Point32>("sonarFilterdata_bag", 1000);
    sonarRawdata_bag = nh_.advertise<geometry_msgs::Point32>("sonarRawdata_bag", 1000);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &argtSLAM::joyCallback, this);
    sonar_sub_ = nh_.subscribe<sensor_msgs::PointCloud>("/RosAria/sonar", 1000, &argtSLAM::sonarChatterCallback, this);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/bruno/occupancy_map", 1, true);
    yawPub = nh_.advertise<std_msgs::Float32>("yawPub", 1);

    std::string frame_id_sonar = "sonar";
    pointCLoudVector_filter.header.frame_id = frame_id_sonar;
    pointCLoudVector_raw.header.frame_id = frame_id_sonar;

    gridMap.resize(1000, std::vector<int>(1000, 0));
    initSampleSonar();
}

// Callback de joystick
void argtSLAM::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_ * joy->axes[angular_];
    twist.linear.x = l_scale_ * joy->axes[linear_];
    vel_pub_.publish(twist);

 
}

// Callback de sonar
void argtSLAM::sonarChatterCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
      geometry_msgs::Point32 p[8];
    geometry_msgs::Point32 p_filter;
    float filter_xk, filter_yk;

    geometry_msgs::Point32 p_r0;
    ROS_INFO("Puntos recibidos:");
    //for (size_t i = 0; i < msg->points.size(); ++i)
    for(int i=0;i<=5;i++)
    {
        const auto &point = msg->points[i];
        double norm = std::sqrt(point.x * point.x + point.y * point.y);
        ROS_INFO("Indice: %ld | x: %.2f | y: %.2f | z: %.2f", i, point.x, point.y, point.z);

        if (norm <= 2.0 && norm != 0)
        {


                    p[i] = point;

        //if( i==0 || i==2 || i==3 || i==5 ){
                    // Transformación a marco global
        p_r0.x = cos(yaw) * p[i].x - sin(yaw) * p[i].y;
        p_r0.y = sin(yaw) * p[i].x + cos(yaw) * p[i].y;
        p_r0.z = 0;

        // Publicación sin filtrar
        sonarRaw.x = p_r0.x + xx;
        sonarRaw.y = p_r0.y + yy;
        sonarRawdata_bag.publish(sonarRaw);

        // Aplicar filtro FIR
        firFilter(i,p_r0.x, p_r0.y, &filter_xk, &filter_yk);
        p_filter.x = filter_xk + xx;
        p_filter.y = filter_yk + yy;

        sonarFilter.x = p_filter.x;
        sonarFilter.y = p_filter.y;
        sonarFilterdata_bag.publish(sonarFilter);

        //ROS_INFO("Indice: %d | Raw: (%.2f, %.2f) | Filtro: (%.2f, %.2f)", i, p_r0.x, p_r0.y, p_filter.x, p_filter.y);

        // Agregar a nubes de puntos
       
            pointCLoudVector_filter.points.push_back(p_filter);
            pointCLoudVector_raw.points.push_back(sonarRaw);

            // Convertir a celda de grid    sonarPoincloud_filter.publish(pointCLoudVector_filter);
            sonarPoincloudraw.publish(pointCLoudVector_raw);
        
 
    publishMap();
        int xGrid, yGrid;
        xy2Grid(sonarRaw.x, sonarRaw.y, xGrid, yGrid);
        if (xGrid != -1 && yGrid != -1)
            fillGrid(xGrid, yGrid);

        }

            sonarPoincloud_filter.publish(pointCLoudVector_filter);
    sonarPoincloudraw.publish(pointCLoudVector_raw);
    publishMap();
                    


        }
    //}





}

// Callback de odometría
void argtSLAM::odomchatterCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    xx = msg->pose.pose.position.x;
    yy = msg->pose.pose.position.y;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);

    double theta_act = yaw;
    if (theta_act - theta_ant > M_PI) numRotation++;
    else if (theta_act - theta_ant < -M_PI) numRotation--;

    double theta_ramp = (2 * M_PI * numRotation + theta_act) * corFactor;
    theta_ant = theta_act;

    double theta_t = fmod(theta_act, 2 * M_PI);
    if (theta_t > M_PI) theta_t -= 2 * M_PI;
    else if (theta_t < -M_PI) theta_t += 2 * M_PI;

    yaw = theta_t;

    yawAmigobot.data = yaw;

    yawPub.publish(yawAmigobot);

    ROS_INFO("Pose actual: x = %.2f | y = %.2f | yaw = %.2f rad", xx, yy, yaw);
}

// Filtro FIR
void argtSLAM::firFilter(int numSonar,float x, float y, float *filter_xk, float *filter_yk)
{
    for (int i = 40; i > 0; --i)
    {
        samplesSonar_x[numSonar][i] = samplesSonar_x[numSonar][i - 1];
        samplesSonar_y[numSonar][i] = samplesSonar_y[numSonar][i - 1];
    }
    samplesSonar_x[numSonar][0] = x;
    samplesSonar_y[numSonar][0] = y;

    *filter_xk = 0.0f;
    *filter_yk = 0.0f;
    for (int i = 0; i < 41; ++i)
    {
        *filter_xk += coefficients[i] * samplesSonar_x[numSonar][i];
        *filter_yk += coefficients[i] * samplesSonar_y[numSonar][i];
    }
}

// Conversión de coordenadas a celdas
void argtSLAM::xy2Grid(const double &x, const double &y, int &xGrid, int &yGrid)
{
    const double gridResolution = 0.1;
    const int gridCenter = 500;

    xGrid = static_cast<int>(std::floor(x / gridResolution)) + gridCenter;
    yGrid = static_cast<int>(std::floor(y / gridResolution)) + gridCenter;
}

// Marcado en el mapa
void argtSLAM::fillGrid(int xGrid, int yGrid)
{
    if (xGrid >= 0 && xGrid < 1000 && yGrid >= 0 && yGrid < 1000)
    {
        gridMap[yGrid][xGrid] = 1;
    }
}

// Publicación del mapa
void argtSLAM::publishMap()
{
    occupancy_grid_.header.stamp = ros::Time::now();
    occupancy_grid_.header.frame_id = "map";
    occupancy_grid_.info.resolution = 0.1;
    occupancy_grid_.info.width = 100;
    occupancy_grid_.info.height = 100;
    occupancy_grid_.info.origin.position.x = -5.0;
    occupancy_grid_.info.origin.position.y = -5.0;
    occupancy_grid_.info.origin.orientation.w = 1.0;

    occupancy_grid_.data.resize(100 * 100);
    for (int y = 0; y < 100; ++y)
    {
        for (int x = 0; x < 100; ++x)
        {
            int index = y * 100 + x;
            occupancy_grid_.data[index] = gridMap[y][x] ? 100 : 0;
        }
    }

    map_pub_.publish(occupancy_grid_);
}

void argtSLAM::initSampleSonar()
{
      for(int i=0;i<8;i++){
    for(int j=0;j<41;j++){
      samplesSonar_x[i][j] = 0;
      samplesSonar_y[i][j] = 0;
    }
  }
}

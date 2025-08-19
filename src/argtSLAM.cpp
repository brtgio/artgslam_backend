#include "argtSlam.hpp"

// Constructor
argtSLAM::argtSLAM()
    :auxpub(nh), linear_(1), angular_(0), l_scale_(0.5), a_scale_(0.5),
      yaw(0.0), x(0.0f), y(0.0f), theta(0.0f),
      theta_previus(0.0), theta_now(0.0), completed_rotations(0)
{
    // Parameters from ROS param server
    nh.param("axis_linear", linear_, linear_);
    nh.param("axis_angular", angular_, angular_);
    nh.param("scale_angular", a_scale_, a_scale_);
    nh.param("scale_linear", l_scale_, l_scale_);

    // Subscribers
    sub_odom = nh.subscribe("/RosAria/pose", 1000, &argtSLAM::odomchatterCallback, this);
    joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &argtSLAM::joyCallback, this);
    sonar_sub_ = nh.subscribe<sensor_msgs::PointCloud>("/RosAria/sonar", 1000, &argtSLAM::sonarChatterCallback, this);

    // Publishers
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
    sonarPoincloud_filter = nh.advertise<sensor_msgs::PointCloud>("sonarPublisher_batman_filter", 1000);
    sonarPoincloudraw = nh.advertise<sensor_msgs::PointCloud>("sonarPublisher_brunodias_raw", 1000);
    sonarFilterdata_bag = nh.advertise<geometry_msgs::Point32>("sonarFilterdata_bag", 1000);
    sonarRawdata_bag = nh.advertise<geometry_msgs::Point32>("sonarRawdata_bag", 1000);
    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/bruno/occupancy_map", 1, true);
    yawPub = nh.advertise<std_msgs::Float32>("yawPub", 1);

    // Set frame_id for sonar pointclouds
    std::string frame_id_sonar = "sonar";
    pointCLoudVector_filter.header.frame_id = frame_id_sonar;
    pointCLoudVector_raw.header.frame_id = frame_id_sonar;

    // Init grid map and sonar samples
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
        sonarRaw.x = p_r0.x + x;
        sonarRaw.y = p_r0.y + y;
        sonarRawdata_bag.publish(sonarRaw);

        // Aplicar filtro FIR
        firFilter(i,p_r0.x, p_r0.y, &filter_xk, &filter_yk);
        p_filter.x = filter_xk + x;
        p_filter.y = filter_yk + y;

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

    // Actualizar posición
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    // Extraer orientación del cuaternión
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);
    auxpub.setThetaAux_pub(yaw);
    theta_now = yaw;
    // Ajustar rotaciones acumuladas
    double thetaAct = yaw;
    if (theta_now - theta_previus > M_PI) {
        completed_rotations++;
    } else if (theta_now - theta_previus < -M_PI) {
        completed_rotations--;
    }

    double thetaRamp = (2 * M_PI * completed_rotations + theta_now) * corFactor_;
    auxpub.setThetaRamp(thetaRamp);
    theta_previus = theta_now;

    // Normalizar ángulo entre -π y π
    double thetaNorm = fmod(thetaRamp, 2 * M_PI);
    if (thetaNorm > M_PI) {
        thetaNorm -= 2 * M_PI;
    } else if (thetaNorm < -M_PI) {
        thetaNorm += 2 * M_PI;
    }

    yaw = thetaNorm;
    auxpub.thetaCorrect(yaw);



    // Log de la pose
    ROS_INFO("Pose actual: x = %.2f | y = %.2f | yaw = %.2f rad", x, y, yaw);
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

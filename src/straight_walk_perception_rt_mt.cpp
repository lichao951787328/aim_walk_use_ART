#include <fstream>
#include <iostream>             // std::cout
#include <future>               // std::async, std::future
#include <chrono>               // std::chrono::milliseconds
#include "tcpip_port.h"
#include <thread>
#include <queue>
#include <glog/logging.h>
#include <conio.h>
#include <unistd.h>
#include <Eigen/Geometry>
#include "DTrackSDK.hpp"
#include "type.h"
#include "matplotlibcpp.h"
// 记录相机深度数据和imu数据
std::mutex m, m_tcpip;
Eigen::Matrix4d mark_pose;
int sock_fd,client_fd;

auto getT(const double &px, const double &py, const double &pz, const double &rx, const double &ry, const double &rz)
{
  using namespace Eigen;
  Matrix4d res;
  res.setIdentity();
  res.block<3,3>(0,0) = (AngleAxisd(rz, Vector3d::UnitZ())*AngleAxisd(ry, Vector3d::UnitY())*AngleAxisd(rx, Vector3d::UnitX())).matrix();
  res(0,3) = px;
  res(1,3) = py;
  res(2,3) = pz;
  return res;
}

auto _deg2rad(double degree)
{
  double rad = degree/57.3;
  return rad;
}

void communicate_PC104()
{
    LOG(INFO)<<"enter communicate PC 104 function"<<endl;
    char currentFlag = 'A';
    char lastFlag = 'A';
    clock_t start, finish;
    double duration;
    tcpip_port com;
    com.initial();
    LOG(INFO)<<"tcp ip port initial finish"<<endl;
    while (1)
    {
        com.accept_client();
        if (com.recvData() == -1)
        {
            printf("recv in server error.");
            exit(1);
        }
        LOG(INFO)<<"recev data"<<endl;
        // cout<<"have recev data"<<endl;
        com.analysisBuf(); 
        // perception.publishWindata(0.0, 0.0);
        lastFlag = currentFlag;
        currentFlag = com.getRecvFlag();
        // std::cout<<"current letter is "<<currentFlag<<std::endl;
        // com.resetRecvFlag();
        // std::cout<<"the com recv flag is change to "<<com.getRecvFlag()<<std::endl;
        if (lastFlag == 'A' && currentFlag == 'B')
        {
            LOG(INFO)<<"GOOOOOOOOOOO"<<endl;
            start = clock();
            // 确定mark相对于机器人世界坐标的位姿
            // Eigen::Matrix4d mark_IN_BW = getT(-0.20725, -0.10999, 1.09618, _deg2rad(168.16), _deg2rad(21.49), _deg2rad(-21.75));
            // loc -213.656 -84.426 1099.334 
            // quatw 0.145 quatxyz 0.952 0.212 0.165
            Eigen::Quaterniond q_mark_IN_BW(0.145, 0.952, 0.212, 0.165);

            Eigen::Matrix4d mark_IN_BW = Eigen::Matrix4d::Identity();
            mark_IN_BW.block<3,3>(0,0) = q_mark_IN_BW.toRotationMatrix();
            mark_IN_BW(0,3) = -213.656/1000;
            mark_IN_BW(1,3) = -84.426/1000;
            mark_IN_BW(2,3) = 1099.334/1000;

            Eigen::Matrix4d BW_IN_RW = Eigen::Matrix4d::Identity();
            BW_IN_RW(0, 3) = 0.3;
            Eigen::Matrix4d mark_IN_RW = mark_IN_BW * BW_IN_RW;
            LOG(INFO)<<"mark_IN_RW: "<<endl<<mark_IN_RW<<endl;
            // 确定接收到的mark相对于art世界坐标的姿态
            Eigen::Matrix4d mark_IN_AW = Eigen::Matrix4d::Identity();
            unique_lock<mutex> g(m, std::defer_lock);
            g.lock();
            mark_IN_AW = mark_pose;
            g.unlock();

            LOG(INFO)<<"mark_IN_AW: "<<endl<<mark_IN_AW<<endl;
            // 转换到aim坐标系求取step
            Eigen::Matrix4d AW_IN_RW = mark_IN_AW.inverse() * mark_IN_RW;
            LOG(INFO)<<"AW_IN_RW: "<<endl<<AW_IN_RW<<endl;
            // 台阶朝向
            Eigen::Vector3d direct_AW = Eigen::Vector3d::UnitX();
            Eigen::Vector3d direct_RW = AW_IN_RW.block<3,3>(0,0) * direct_AW;
            LOG(INFO)<<"direct_RW: "<<endl<<direct_RW.transpose()<<endl;
            assert(direct_RW(0)>0 &&  "direct error");
            Eigen::Vector2d goal_AW = AW_IN_RW.block<2,1>(0, 3);
            LOG(INFO)<<"goal_AW: "<<endl<<goal_AW.transpose()<<endl;
            Eigen::Vector2d direct_2d = direct_RW.head(2).normalized();
            double dis_AW = goal_AW.dot(direct_2d);
            LOG(INFO)<<dis_AW<<endl;
            double dis_walk = dis_AW - 0.19;// 前脚掌
            assert(dis_walk>0 && "dis error");
            double theta = acos(abs(direct_RW(0)));
            LOG(INFO)<<"THETA IS "<<theta * 57.3<<endl;
            double goal_dis = dis_walk;

            // 假设为正确的x方向
            // Eigen::Vector4d x_d;
            // x_d.head(3) = Eigen::Vector3d::UnitY();
            // x_d(3) = 1.0;
            // Eigen::Vector3d direct = (World_T_Tar * x_d).head(3);
            // LOG(INFO)<<"DIRECT 3D: "<<direct.transpose()<<endl;
            // Eigen::Vector2d direct_2d = direct.head(2).normalized();
            // LOG(INFO)<<"goal 3d: "<<World_T_Tar.block<3, 1>(0, 3).transpose()<<endl;
            // Eigen::Vector2d goal = World_T_Tar.block<2, 1>(0, 3);
            // LOG(INFO)<<"goal position: "<<goal.transpose()<<endl;
            // double dis_tag = 0.1 + 0.17;// 此为粘贴时测量
            // Eigen::Vector2d walk_goal = goal - dis_tag * direct_2d;
            // // 至此，便得到了方向和目标点

            // // double dis = abs(walk_goal.dot(direct_2d));
            // double goal_dis = walk_goal.norm();
            // // double goal_dis = dis - 0.17;//前脚长15cm + 1cm阈值
            // LOG(INFO)<<"aim direct "<<direct_2d.transpose()<<endl;
            // LOG(INFO)<<"goal distance : "<<goal_dis<<endl;
            // double theta = acos(Eigen::Vector2d::UnitX().dot(direct_2d));
            // CHECK_GE(theta, 0.0)<<"theta is > 0"<<endl;
            // LOG(ERROR)<<"THETA : "<<theta<<endl;
            struct line_step
            {
            double x, y, theta;
            };
            vector<footstep> steps_result;
            if (direct_2d(1) < 0)//右转
            {
                LOG(INFO)<<"TURN RIGHT ..."<<endl;
                int num_foot_len = (int)(goal_dis/0.25 + 0.8);
                int num_foot_angle = (int)(abs(theta)/(6/57.3) + 0.8);
                int num_foot = max(num_foot_len, num_foot_angle);
                double length_step = goal_dis / num_foot;
                double theta_step = theta / num_foot;
                LOG(INFO)<<"step length "<<length_step<<endl;
                LOG(INFO)<<"step angle "<<theta_step<<endl;
                vector<line_step> line_steps;
                line_steps.reserve(num_foot);
                for (size_t i = 0; i < num_foot; i++)
                {
                    Eigen::Vector2d line_cor = length_step *(i+1) *direct_2d;
                    double tmptheta = (i+1) * theta_step;
                    line_step tmp_line_step;
                    tmp_line_step.x = line_cor(0);
                    tmp_line_step.y = line_cor(1);
                    tmp_line_step.theta = tmptheta;
                    line_steps.emplace_back(tmp_line_step);
                }
                LOG(INFO)<<"line steps :"<<endl;
                for (auto & iter_line_step : line_steps)
                {
                    LOG(INFO)<<iter_line_step.x<<" "<<iter_line_step.y<<" "<<iter_line_step.theta<<endl;
                }
                for (auto & iter_line_step : line_steps)
                {
                    Eigen::Vector3d t(iter_line_step.x, iter_line_step.y, 0.0);
                    Eigen::Vector3d left_foot_cor(0.0, 0.08, 0.0);
                    Eigen::Vector3d right_foot_cor(0.0, -0.08, 0.0);
                    Eigen::AngleAxisd rotate_vector( - iter_line_step.theta, Eigen::Vector3d::UnitZ());
                    Eigen::Vector3d left_foot_cor_ro = rotate_vector.toRotationMatrix() * left_foot_cor + t;
                    Eigen::Vector3d right_foot_cor_ro = rotate_vector.toRotationMatrix() * right_foot_cor + t;
                    footstep tmpstep1;
                    tmpstep1.is_left = false;
                    tmpstep1.x = right_foot_cor_ro(0);
                    tmpstep1.y = right_foot_cor_ro(1);
                    tmpstep1.z = right_foot_cor_ro(2);
                    tmpstep1.theta = - iter_line_step.theta;
                    steps_result.emplace_back(tmpstep1);
                    footstep tmpstep2;
                    tmpstep2.is_left = true;
                    tmpstep2.x = left_foot_cor_ro(0);
                    tmpstep2.y = left_foot_cor_ro(1);
                    tmpstep2.z = left_foot_cor_ro(2);
                    tmpstep2.theta = - iter_line_step.theta;
                    steps_result.emplace_back(tmpstep2);
                }
                LOG(INFO)<<"foot step:"<<endl;
                for (auto & iter_footstep : steps_result)
                {
                    LOG(INFO)<<iter_footstep.is_left<<" "<<iter_footstep.x<<" "<<iter_footstep.y<<" "<<iter_footstep.z<<" "<<iter_footstep.theta<<endl;
                }
            }
            else//左转
            {
                LOG(INFO)<<"TURN LEFT ..."<<endl;
                int num_foot_len = (int)(goal_dis/0.25 + 0.8);
                int num_foot_angle = (int)(abs(theta)/(6/57.3) + 0.8);
                int num_foot = max(num_foot_len, num_foot_angle);
                double length_step = goal_dis / num_foot;
                double theta_step = theta / num_foot;
                LOG(INFO)<<"step length "<<length_step<<endl;
                LOG(INFO)<<"step angle "<<theta_step<<endl;
                vector<line_step> line_steps;
                line_steps.reserve(num_foot);
                steps_result.reserve(num_foot * 2);
                for (size_t i = 0; i < num_foot; i++)
                {
                    Eigen::Vector2d line_cor = length_step *(i+1) *direct_2d;
                    double tmptheta = (i+1) * theta_step;
                    line_step tmp_line_step;
                    tmp_line_step.x = line_cor(0);
                    tmp_line_step.y = line_cor(1);
                    tmp_line_step.theta = tmptheta;
                    line_steps.emplace_back(tmp_line_step);
                }
                for (auto & iter_line_step : line_steps)
                {
                    LOG(INFO)<<iter_line_step.x<<" "<<iter_line_step.y<<" "<<iter_line_step.theta<<endl;
                }
                for (auto & iter_line_step : line_steps)
                {
                    Eigen::Vector3d t(iter_line_step.x, iter_line_step.y, 0.0);
                    Eigen::Vector3d left_foot_cor(0.0, 0.08, 0.0);
                    Eigen::Vector3d right_foot_cor(0.0, -0.08, 0.0);
                    Eigen::AngleAxisd rotate_vector(iter_line_step.theta, Eigen::Vector3d::UnitZ());
                    Eigen::Vector3d left_foot_cor_ro = rotate_vector.toRotationMatrix() * left_foot_cor + t;
                    Eigen::Vector3d right_foot_cor_ro = rotate_vector.toRotationMatrix() * right_foot_cor + t;
                    footstep tmpstep1;
                    tmpstep1.is_left = true;
                    tmpstep1.x = left_foot_cor_ro(0);
                    tmpstep1.y = left_foot_cor_ro(1);
                    tmpstep1.z = left_foot_cor_ro(2);
                    tmpstep1.theta = iter_line_step.theta;
                    steps_result.emplace_back(tmpstep1);
                    footstep tmpstep2;
                    tmpstep2.is_left = false;
                    tmpstep2.x = right_foot_cor_ro(0);
                    tmpstep2.y = right_foot_cor_ro(1);
                    tmpstep2.z = right_foot_cor_ro(2);
                    tmpstep2.theta = iter_line_step.theta;
                    steps_result.emplace_back(tmpstep2);
                }
                LOG(INFO)<<"foot step:"<<endl;
                for (auto & iter_footstep : steps_result)
                {
                    LOG(INFO)<<iter_footstep.is_left<<" "<<iter_footstep.x<<" "<<iter_footstep.y<<" "<<iter_footstep.z<<" "<<iter_footstep.theta<<endl;
                }
            }
            bool return_flag = steps_result.size() > 0;
            finish = clock();
            duration = (double)(finish - start) / CLOCKS_PER_SEC;
            cout<<"plane detection and step planning cost "<<duration<<endl;
            LOG(INFO)<<"step size "<<steps_result.size()<<endl;
            // 画步态点
            // for (auto step_iter : steps_result)
            // {
            //     std::array<Eigen::Vector3d, 4> edge_points;// 27 10 5 14.5
            //     edge_points.at(0) = Eigen::Vector3d(0.17, -0.05, 0.0);
            //     edge_points.at(1) = Eigen::Vector3d(0.17,  0.095, 0.0);
            //     edge_points.at(2) = Eigen::Vector3d(- 0.1, 0.095, 0.0);
            //     edge_points.at(3) = Eigen::Vector3d(-0.1, -0.05, 0.0);
            //     LOG(INFO)<<"GET EDGE POINTS"<<endl;
            //     Eigen::AngleAxisd r_v(step_iter.theta, Eigen::Vector3d(0,0,1));
            //     std::array<Eigen::Vector3d, 4> draw_points;
            //     std::vector<double> x, y;
            //     x.reserve(4); y.reserve(4);
            //     for (size_t i = 0; i < 4; i++)
            //     {
            //         draw_points.at(i) = r_v.matrix() * edge_points.at(i) + Eigen::Vector3d(step_iter.x, step_iter.y, step_iter.z);
            //         x.emplace_back(draw_points.at(i)(0));
            //         y.emplace_back(draw_points.at(i)(1));
            //     }
            //     LOG(INFO)<<"get x and y"<<endl;
            //     matplotlibcpp::plot(y, x);
            // }
            
            // matplotlibcpp::show();

            if (!return_flag)
            {
                LOG(INFO)<<"perception wrong, send an empty data to win. "<<endl;
                int buf_size = sizeof(int);
                char* senddata = new char[buf_size];
                int steps_num = 0;
                memcpy((void*)senddata, (void*)(&steps_num), sizeof(int));
                if (com.sendSteps(senddata, buf_size) == -1)
                {
                    perror("send error");
                    exit(1);
                }
                cout<<"has send 0 to win, you can ansys it use type int at buffer"<<endl;
            }
            else
            {
                vector<footstep> send_steps = steps_result;
                // send_steps.emplace_back(perception.steps_result.at(2));
                // data to file
                if (send_steps.size() <= 2)
                {
                    LOG(INFO)<<"some logic error occured in map or stepplanning, please check..."<<endl;
                }
                
                // for (size_t i = 0; i < perception.steps_result.size(); i++)
                // {
                //     fs_communicate<<"step "<<i+1<<": ";
                //     fs_communicate<<perception.steps_result.at(i).is_left<<" "<<perception.steps_result.at(i).x<<" "<<perception.steps_result.at(i).z<<endl;
                // }
                
                //  = perception.steps_result;
                LOG(INFO)<<"STEP PLANNING RESULTS:"<<endl;
                for (auto & iter_step : send_steps)
                {
                    LOG(INFO)<<iter_step.is_left<<" "<<iter_step.x<<" "<<iter_step.y<<" "<<iter_step.z<<" "<<iter_step.theta<<endl;
                }
                // 改了最大步数，16步
                struct sendddata
                {
                    int n;
                    footstep steps[16];
                };

                sendddata SENDdata;
                SENDdata.n = send_steps.size();
                for (size_t i = 0; i < send_steps.size(); i++)
                {
                    SENDdata.steps[i].is_left = send_steps.at(i).is_left;
                    SENDdata.steps[i].x = send_steps.at(i).x;
                    SENDdata.steps[i].y = send_steps.at(i).y;
                    SENDdata.steps[i].z = send_steps.at(i).z;
                    SENDdata.steps[i].theta = send_steps.at(i).theta;

                }
                cout<<"size of sendddata "<<sizeof(sendddata)<<endl;
                if (com.sendSteps((char*)(&SENDdata), sizeof(sendddata)) == -1)
                {
                    perror("send error");
                    exit(1);
                }

                // DRAW
                
            }
            lastFlag = 'A';
            currentFlag = 'A';
        }
        else
        {
            LOG(INFO)<<"do not need to perception, send an empty data to win. "<<endl;
            int buf_size = sizeof(int);
            char* senddata = new char[buf_size];
            int steps_num = 0;
            memcpy((void*)senddata, (void*)(&steps_num), sizeof(int));
            if (com.sendSteps(senddata, buf_size) == -1)
            {
                perror("send error");
                exit(1);
            }
            // cout<<"has send 0 to win"<<endl;
        }
        LOG(INFO)<<"last flag is "<<lastFlag<<endl;
        LOG(INFO)<<"current flag is "<<currentFlag<<endl;
        com.close_client();
        LOG(INFO)<<"CLOSE CLIENT FINISH"<<endl;
    }
}

int communicate_ART()
{
    // 此处初始化
    LOG(INFO)<<"listen art data..."<<endl;
    unsigned short port = 6324;

	// initialization:
    static DTrackSDK* dt = NULL;
	dt = new DTrackSDK( port );

	if ( ! dt->isDataInterfaceValid() )
	{
		std::cout << "DTrackSDK init error" << std::endl;
		return -3;
	}
	std::cout << "listening at local data port " << dt->getDataPort() << std::endl;
	// measurement:
    while (1)
    {
        if(dt->receive())
        {
            std::cout.precision( 3 );
            std::cout.setf( std::ios::fixed, std::ios::floatfield );

            // std::cout << std::endl << "frame " << dt->getFrameCounter() << " ts " << dt->getTimeStamp()
            //         << " nbod " << dt->getNumBody() << " nfly " << dt->getNumFlyStick()
            //         << " nmea " << dt->getNumMeaTool() << " nmearef " << dt->getNumMeaRef() 
            //         << " nhand " << dt->getNumHand() << " nmar " << dt->getNumMarker() 
            //         << " nhuman " << dt->getNumHuman() << " ninertial " << dt->getNumInertial()
            //         << std::endl;

            const DTrackBody* body = dt->getBody( 1 );
            if ( body == NULL )
            {
                std::cout << "DTrackSDK fatal error: invalid body id " << 0 << std::endl;
                break;
            }

            if ( ! body->isTracked() )
            {
                std::cout << "bod " << body->id << " not tracked" << std::endl;
            }
            else
            {
                // std::cout << "bod " << body->id << " qu " << body->quality
                //         << " loc " << body->loc[ 0 ] << " " << body->loc[ 1 ] << " " << body->loc[ 2 ]
                //         << " rot " << body->rot[ 0 ] << " " << body->rot[ 1 ] << " " << body->rot[ 2 ]
                //         << " "     << body->rot[ 3 ] << " " << body->rot[ 4 ] << " " << body->rot[ 5 ]
                //         << " "     << body->rot[ 6 ] << " " << body->rot[ 7 ] << " " << body->rot[ 8 ]
                //         << std::endl;

                unique_lock<mutex> g(m, std::defer_lock);
                g.lock();
                mark_pose.setIdentity();
                mark_pose<<body->rot[0], body->rot[3], body->rot[6], body->loc[0]/1000,
                            body->rot[1], body->rot[4], body->rot[7], body->loc[1]/1000,
                            body->rot[2], body->rot[5], body->rot[8], body->loc[2]/1000,
                            0,            0,             0,           1;
                g.unlock();
                // std::cout<<mark_pose<<endl;
                DTrackQuaternion quat = body->getQuaternion();
                // std::cout << "bod " << body->id << " quatw " << quat.w
                //         << " quatxyz " << quat.x << " " << quat.y << " " << quat.z << std::endl;
                Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
                // std::cout<<q.toRotationMatrix()<<std::endl;
            }
        }
        else
        {
            if ( dt->getLastDataError() == DTrackSDK::ERR_TIMEOUT )
            {
                std::cout << "--- timeout while waiting for tracking data" << std::endl;
                return -1;
            }

            if ( dt->getLastDataError() == DTrackSDK::ERR_NET )
            {
                std::cout << "--- error while receiving tracking data" << std::endl;
                return -1;
            }

            if ( dt->getLastDataError() == DTrackSDK::ERR_PARSE )
            {
                std::cout << "--- error while parsing tracking data" << std::endl;
                return -1;
            }
        }
    }
	delete dt;  // clean up
	return 0;
}

void exitall()
{
    LOG(INFO)<<"enter exit all function"<<endl;
    int ch;
    while (1){
		if (_kbhit())
        {//如果有按键按下，则_kbhit()函数返回真
            LOG(INFO)<<"PRESS KEY"<<endl;
			ch = getch();//使用_getch()获取按下的键值
            LOG(INFO)<<"GET CHAR "<<ch<<endl;
			cout << ch;
			if (ch == 27)
            { 
                close(sock_fd);
                close(client_fd);
                LOG(INFO)<<"CLOSE TCPIP PORT..."<<endl;
                break; 
                //当按下ESC时退出循环，ESC键的键值是27.
            }
        }
	}
}

int main(int argc, char** argv)
{
    // google::ParseCommandLineFlags(&argc, &argv, true); 
    google::InitGoogleLogging(argv[0]); 
    FLAGS_colorlogtostderr = true;
    FLAGS_log_dir = "./log"; 
    FLAGS_alsologtostderr = true;
    LOG(INFO)<<"initial glog finish"<<endl;
    LOG(WARNING)<<"WARNING"<<endl;

    thread th1(exitall);
    thread th2(communicate_PC104);
    thread th3(communicate_ART);
    
    th1.join();
    th2.join();
    th3.join();
    google::ShutdownGoogleLogging();
    return 0;
}


/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the 
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0 
 *
 * Contributors: 
 *    Reza Dariani- initial implementation and API
 ********************************************************************************/
#pragma once

#include <adore/env/map/occupancy_grid.h>
#include <eigen3/Eigen/Core>
#include <adore/fun/node.h>
#include <adore/fun/search_grid.h>
#include <adore/fun/hybrid_A_star.h>

#include <nav_msgs/OccupancyGrid.h>
#include <plotlablib/figurestubfactory.h>
#include <adore/fun/collision_check_offline.h>
//#include <adore/fun/vornoi_diagram.h>
#include <plotlablib/afigurestub.h>
#include <geometry_msgs/Pose.h>
//#include <adore/mad/catmull_rom_splines.h>
#include <boost/geometry.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <adore/fun/trajectory_smoothing.h>
#include <ctime>
#include <chrono>
#include <stdint.h>
#include <tf/tf.h>
#include <iterator>
//#include <tf/quaterion.h>

namespace adore
{
namespace apps
{
    /**
     * 
     */
    class GraphSearch
    {
        public:        //typedef boost::geometry::model::point<double,2,boost::geometry::cs::cartesian> Vector;
            DLR_TS::PlotLab::FigureStubFactory fig_factory;
            DLR_TS::PlotLab::AFigureStub* figure3;  
            DLR_TS::PlotLab::AFigureStub* figure4; 
            DLR_TS::PlotLab::AFigureStub* figure5;         
            int time1, time2;
            std::chrono::system_clock::time_point  start;
            std::chrono::system_clock::time_point  end;        
            int grid_width = 80; //73;
            int grid_height = 20;//20;  
            static const int HeadingResolution = 30;  //5
            static const int nH_Type = 3;  //non holonomic
            static const int H_Type = 2;  //holonomic
            int Depth;      
            adore::env::OccupanyGrid OG;
            adore::fun::GRID<adore::fun::Node<nH_Type,double>> NH_GRID;
            ros::Subscriber StartPose_subscreiber; 
            ros::Subscriber EndPose_subscreiber; 
            //adore::fun::ArrayFormGrid<adore::fun::Node<nH_Type,double>> NH_GRID;
            adore::fun::Hybrid_A_Star* h_A_star;
            adore::fun::Node<3,double> Start;
            adore::fun::Node<3,double> End;
            bool validStart, validEnd;
            adore::fun::CollisionCheckOffline* cco;
            ros::NodeHandle* node_;
            fun::TrajectorySmoothing* smoothing;
            double avg_time;
            float vehicleLength = 0.0;
            float vehicleWidth = 0.0;
            int iteration;
            GraphSearch(const nav_msgs::OccupancyGrid::ConstPtr &msg, int test, uint32_t height, uint32_t width, ros::NodeHandle* parentnode)
            {
                std::cout<<"msg data"<<std::endl;
                std::cout<<"\n \n init search"<<std::endl;
                grid_height = height;
                grid_width = width;  
                //std::cout<< "test" <<std::endl;
                vehicleLength = 3.2;
                vehicleWidth = 1.0; 
                //std::cout<<"start smooth init"<<std::endl;
                smoothing = new fun::TrajectorySmoothing;
                //std::cout<<"start a_start init"<<std::endl;
                h_A_star = new adore::fun::Hybrid_A_Star(smoothing);
                //std::cout<<"start fig init tesst"<<std::endl;
                node_ = parentnode;
                std::cout<<"heigth: " << height << "  width: " << width << std::endl;
            
            
                figure3 = fig_factory.createFigureStub(3);
                //std::cout<<"fig3"<<std::endl;
                figure3->showAxis();
                figure3->showGrid();
                figure3->show();  
                figure4 = fig_factory.createFigureStub(4);
                //std::cout<<"fig4"<<std::endl;
                figure4->showAxis();
                figure4->showGrid();
                figure4->show();   
                figure5 = fig_factory.createFigureStub(5);
                figure5->showAxis();
                figure5->showGrid();
                figure5->show();              
                Depth = 360 / HeadingResolution;
                cco = new adore::fun::CollisionCheckOffline(2, 2, HeadingResolution, 10);
                std ::cout<<"cco init"<<std::endl;
                std::cout<<height << width <<std::endl;
                
                OG.init(msg, grid_height, grid_width);
                OG.resize(grid_height,grid_width,figure3);
                std ::cout<<"og init"<<std::endl;
                NH_GRID.resize(height,width,Depth);
                std ::cout<<"nh grid init"<<std::endl;
                h_A_star->setSize(height,width);
                avg_time = 0.0;
                iteration = 1;
                std::cout<<"init search algo end"<<std::endl;
                

            }
            void update()
        {
            
            StartPose_subscreiber= node_->subscribe<geometry_msgs::Pose>("StartPose",1,&GraphSearch::receiveStartPose,this);
            EndPose_subscreiber= node_->subscribe<geometry_msgs::Pose>("EndPose",1,&GraphSearch::receiveEndPose,this);


            while(iteration<2 && validStart && validEnd)
            {

                std::cout<<"\n ITERATION: "<<iteration;

                start = std::chrono::system_clock::now();
                time1 = 0.0;
                time2 = 0.0;
                std::cout<<"next iteration"<<std::endl;
                   
                //std::cout<<"\n"<<   cco->offlineCollisionTable.size()<<"\t"<<cco->offlineCollisionTable[0].size1()<<"\t"<<cco->offlineCollisionTable[0].size2();   
                h_A_star->plan(&NH_GRID,&OG, cco, &Start,&End,HeadingResolution,1000, vehicleWidth, vehicleLength ,figure3,figure4,figure5);            
                end = std::chrono::system_clock::now(); 
                time1 += std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();   
                time2 += std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); 
                avg_time += time1;
                std::cout << "Elapsed time in microseconds (profiler) : "<< time1  << "\n";  
                //std::cout << "Elapsed time in milliseconds (profiler) : "<< time2  << "\n";    
                std::cout << "Average time                 (profiler) : "<< (avg_time/iteration)/1000  << "\n";  
                iteration++;          
                
            }

        }

        void receiveStartPose(geometry_msgs::Pose msg)
            {
                        double r,p,y;
                        tf::Matrix3x3(tf::Quaternion(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)).getRPY(r,p,y);
                        if(OG.check_valid_position(msg.position.y,msg.position.x)){
                            validStart = Start.setPosition(msg.position.x,msg.position.y,y,grid_height,grid_width,Depth,adore::mad::CoordinateConversion::DegToRad(HeadingResolution), figure3);
                        }
                        else{
                            std::cout<<"Invalid Start Pose"<<std::endl;
                        }
                        //Start.print();
            }  
        void receiveEndPose(geometry_msgs::Pose msg)
            {
                        double r,p,y;
                        tf::Matrix3x3(tf::Quaternion(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)).getRPY(r,p,y);
                        if(OG.check_valid_position(msg.position.y,msg.position.x)){
                            validEnd = End.setPosition(msg.position.x,msg.position.y,y,grid_height,grid_width,Depth, adore::mad::CoordinateConversion::DegToRad(HeadingResolution),  figure3);
                        }
                        else{
                            std::cout<<"Invalid End Pose"<<std::endl;
                        }
                        //End.print();
            }               
    };
}
}



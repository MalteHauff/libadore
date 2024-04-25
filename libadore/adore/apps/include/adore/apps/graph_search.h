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

namespace adore
{
namespace apps
{
    /**
     * 
     */
    class GraphSearch
    {
        private:
        //typedef boost::geometry::model::point<double,2,boost::geometry::cs::cartesian> Vector;
            DLR_TS::PlotLab::FigureStubFactory fig_factory;
            DLR_TS::PlotLab::AFigureStub* figure3;  
            DLR_TS::PlotLab::AFigureStub* figure4; 
            DLR_TS::PlotLab::AFigureStub* figure5;         
            int time1, time2;
            std::chrono::system_clock::time_point  start;
            std::chrono::system_clock::time_point  end;        
            static const int Length = 80; //73;
            static const int Width = 20;//20;  
            static const int HeadingResolution = 5;  
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
            double vehicleLength, vehicleWidth;
            int iteration;

        
        public:
        GraphSearch(ros::NodeHandle* parentnode)
        {
            vehicleLength = 3.2;
            vehicleWidth = 1.0; 
            smoothing = new fun::TrajectorySmoothing;
            h_A_star = new adore::fun::Hybrid_A_Star(smoothing);
            node_ = parentnode;
            figure3 = fig_factory.createFigureStub(3);
            figure3->showAxis();
            figure3->showGrid();
            figure3->show();  
            figure4 = fig_factory.createFigureStub(4);
            figure4->showAxis();
            figure4->showGrid();
            figure4->show();   
            figure5 = fig_factory.createFigureStub(5);
            figure5->showAxis();
            figure5->showGrid();
            figure5->show();                               
            Depth = 360 / HeadingResolution;
            cco = new adore::fun::CollisionCheckOffline(vehicleWidth, vehicleLength, HeadingResolution, 10);
            OG.resize(Width,Length,figure3);
            NH_GRID.resize(Width,Length,Depth);
            h_A_star->setSize(Width,Length);
            avg_time = 0.0;
            iteration = 1;
            


        }
        void update()
        {
        //--------------------------
               
       
        //-------------------------

       //adore::fun::VornoiDiagram::update(figure);
       
            StartPose_subscreiber= node_->subscribe<geometry_msgs::Pose>("SIM/StartPose",1,&GraphSearch::receiveStartPose,this);
            EndPose_subscreiber= node_->subscribe<geometry_msgs::Pose>("SIM/EndPose",1,&GraphSearch::receiveEndPose,this);
            
            while(iteration<2 && validStart && validEnd)
            {

                std::cout<<"\nITERATION: "<<iteration;
                start = std::chrono::system_clock::now();
                time1 = 0.0;
                time2 = 0.0;
                   
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
                    validStart = Start.setPosition(msg.position.x,msg.position.y,y,Width,Length,Depth,adore::mad::CoordinateConversion::DegToRad(HeadingResolution), figure3);
                    //Start.print();
        }  
        void receiveEndPose(geometry_msgs::Pose msg)
        {
                    double r,p,y;
                    tf::Matrix3x3(tf::Quaternion(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)).getRPY(r,p,y);
                    validEnd = End.setPosition(msg.position.x,msg.position.y,y,Width,Length,Depth, adore::mad::CoordinateConversion::DegToRad(HeadingResolution),  figure3);
                    //End.print();
        }              
    };
}
}


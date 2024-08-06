#include "Planners/AStar_EDF.hpp"

namespace Planners{
    
AStarEDF::AStarEDF(bool _use_3d):AStar(_use_3d, "astar_edf") {}
AStarEDF::AStarEDF(bool _use_3d, std::string _name = "astar_edf" ):AStar(_use_3d, _name) {}

// This computeG is considering the distance cost.
inline unsigned int AStarEDF::computeG(const Node* _current, Node* _suc, unsigned int _n_i, unsigned int _dirs){
    
    unsigned int cost = _current->G;

    if(_dirs == 8){
        cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
    }else{
        cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
    }
 
    // auto cost_term = static_cast<unsigned int>(cost_weight_ * _suc->cost * dist_scale_factor_reduced_);
    
    // std::cout << "grad " << grad << std::endl;
    // std::cout << "cost_weight " << cost_weight_ << std::endl;
    // cost_weight_ = 1;
    // auto cost_term = static_cast<unsigned int>(cost_weight_ * ((_current->cost + _suc->cost)/2) * dist_scale_factor_reduced_); //+grad_suc
    auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_current->cost) + static_cast<double>(_suc->cost))/2) * dist_scale_factor_reduced_))); 
    // std::cout << "cost_term " << cost_term << std::endl;
    // std::cout << "cost " << _suc->cost << std::endl;
    std::cout << "semantic " << _suc->semantic << std::endl;
    cost += cost_term;
    // std::cout << "cost after " << cost << std::endl;
    _suc->C = cost_term;
    
    return cost;
}

// // This computeG is the original AStar
// inline unsigned int AStarEDF::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs){
//     unsigned int cost = _current->G;

//     if(_dirs  == 8){
//         cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
//     }else{
//         cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
//     }
    
//     _suc->C = _suc->cost;
    
//     return cost;
// }



void AStarEDF::exploreNeighbours_Gradient(Node* _current, const Vec3i &_target,node_by_position &_index_by_pos){

        // EXT: Check which node is each i.
        // ROS_INFO("GRADIENT");

        angles_h[0]=1.5708;
        angles_v[0]=0;
        angles_h[1]=-1.5708;
        angles_v[1]=0;
        angles_h[2]=0;
        angles_v[2]=0;
        angles_h[3]=3.14159;
        angles_v[3]=0;
        angles_h[4]=0;
        angles_v[4]=1.5708;
        angles_h[5]=0;
        angles_v[5]=-1.5708;
        angles_h[6]=-0.785398;
        angles_v[6]=0;
        angles_h[7]=2.35619;
        angles_v[7]=0;
        angles_h[8]=-2.35619;
        angles_v[8]=0;
        angles_h[9]=0.785398;
        angles_v[9]=0;
        angles_h[10]=3.14159;
        angles_v[10]=-0.788391;
        angles_h[11]=0;
        angles_v[11]=0.788391;
        angles_h[12]=0;
        angles_v[12]=-0.788391;
        angles_h[13]=3.14159;
        angles_v[13]=0.788391;
        angles_h[14]=-1.5708;
        angles_v[14]=0.788391;
        angles_h[15]=1.5708;
        angles_v[15]=0.788391;
        angles_h[16]=1.5708;
        angles_v[16]=-0.788391;
        angles_h[17]=-1.5708;
        angles_v[17]=-0.788391;
        angles_h[18]=-2.35619;
        angles_v[18]=0.616318;
        angles_h[19]=0.785398;
        angles_v[19]=0.616318;
        angles_h[20]=2.35619;
        angles_v[20]=0.616318;
        angles_h[21]=-0.785398;
        angles_v[21]=0.616318;
        angles_h[22]=-2.35619;
        angles_v[22]=-0.616318;
        angles_h[23]=0.785398;
        angles_v[23]=-0.616318;
        angles_h[24]=2.35619;
        angles_v[24]=-0.616318;
        angles_h[25]=-0.785398;
        angles_v[25]=-0.616318;
        
        // Compute the gradient of each node and the total one
        // const int tam_grad=direction.size();
        float gradient[26];
        Vec3i gradient_vector[26];
        Vec3f gradient_unit_vector[26];

        // Compute unit vector of the goal from each successor
        // Vec3i attraction[26];
        // Vec3f attraction_unit[26];
        
        Vec3i attraction_current[1];
        Vec3f attraction_unit_current[1];
        float module_current;
        
        // Compute unit vector of the goal from the current
        Vec3i current_h = _current->coordinates;
        Node *current_grad = discrete_world_.getNodePtr(current_h);
        attraction_current[0]=getVectorPull(current_grad->coordinates, _target);
        module_current=sqrt(pow(attraction_current[0].x, 2) + pow(attraction_current[0].y, 2) + pow(attraction_current[0].z, 2));
        attraction_unit_current[0].x = attraction_current[0].x/module_current;
        attraction_unit_current[0].y = attraction_current[0].y/module_current;
        attraction_unit_current[0].z = attraction_current[0].z/module_current;

        // std::cout << "current X: " << attraction_unit_current[0].x << std::endl;
        // std::cout << "current Y: " << attraction_unit_current[0].y << std::endl;
        // std::cout << "current Z: " << attraction_unit_current[0].z << std::endl;

        // Computation of each gradient
        for (unsigned int i = 0; i < direction.size(); ++i) {

            Vec3i newCoordinates_grad = _current->coordinates + direction[i];
            Node *successor_grad = discrete_world_.getNodePtr(newCoordinates_grad);
            
            // float module;
            float module_grad;

            if ( successor_grad == nullptr || 
                 successor_grad->occuppied ) 
                continue;
            
            if ( successor_grad->isInClosedList ){
                    // ThetaStar2
                    // gradient[i]=-1;
                    // ThetaStar1
                    // gradient[i]=500*(computeGradient(_current, successor_grad, i, direction.size())); //500 is given by 100 and 0.2 (res) --> 100/0.2
                    gradient[i]=computeGradient(_current, successor_grad, i, direction.size()); //500 is given by 100 and 0.2 (res) --> 100/0.2
                    gradient_vector[i]=getVectorPull(current_grad->coordinates, successor_grad->coordinates);

                    module_grad=0;
                    module_grad = sqrt(pow(gradient_vector[i].x, 2) + pow(gradient_vector[i].y, 2) + pow(gradient_vector[i].z, 2));
                    gradient_unit_vector[i].x = gradient_vector[i].x/module_grad;
                    gradient_unit_vector[i].y = gradient_vector[i].y/module_grad;
                    gradient_unit_vector[i].z = gradient_vector[i].z/module_grad;
                    // continue;
                 }
            else { 
                
                // gradient[i]=500*(computeGradient(_current, successor_grad, i, direction.size())); //500 is given by 100 and 0.2 (res) --> 100/0.2
                gradient[i]=(computeGradient(_current, successor_grad, i, direction.size())); //500 is given by 100 and 0.2 (res) --> 100/0.2
                gradient_vector[i]=getVectorPull(current_grad->coordinates, successor_grad->coordinates);
                
                module_grad=0;
                module_grad = sqrt(pow(gradient_vector[i].x, 2) + pow(gradient_vector[i].y, 2) + pow(gradient_vector[i].z, 2));
                gradient_unit_vector[i].x = gradient_vector[i].x/module_grad;
                gradient_unit_vector[i].y = gradient_vector[i].y/module_grad;
                gradient_unit_vector[i].z = gradient_vector[i].z/module_grad;

                // std::cout << "gradient_vector X: " << gradient_unit_vector[i].x << std::endl;
                // std::cout << "gradient_vector Y: " << gradient_unit_vector[i].y << std::endl;
                // std::cout << "gradient_vector Z: " << gradient_unit_vector[i].z << std::endl;

                // Compute unit vector of the goal from each successor NOW IT IS USED THE ATTRACTION FROM THE CURRENT
                // module = 0;
                // attraction[i]=getVectorPull(successor_grad->coordinates, _target);
                // module = sqrt(pow(attraction[i].x, 2) + pow(attraction[i].y, 2) + pow(attraction[i].z, 2));
                // // std::cout << "module: " << module << std::endl;
                // // std::cout << "attraction X: " << attraction[i].x << std::endl;
                // // std::cout << "attraction Y: " << attraction[i].y << std::endl;
                // // std::cout << "attraction Z: " << attraction[i].z << std::endl;

                // attraction_unit[i].x = attraction[i].x/module;
                // attraction_unit[i].y = attraction[i].y/module;
                // attraction_unit[i].z = attraction[i].z/module;
                // std::cout << "attraction_unit X: " << attraction_unit[i].x << std::endl;
                // std::cout << "attraction_unit Y: " << attraction_unit[i].y << std::endl;
                // std::cout << "attraction_unit Z: " << attraction_unit[i].z << std::endl;

                // std::cout << "current: " << _current->cost << std::endl;
                // std::cout << "suc: " << successor_grad->cost << std::endl;
                // std::cout << "Cell: " << i+1 << std::endl;
                // std::cout << "gradient: " << gradient[i] << std::endl;
            }
        }

        // Compute unit vector of the gradient
        float max_gradient=0;
        int index_max_gradient=0;
        for (unsigned int i = 0; i < direction.size(); ++i) {
            if (gradient[i] > max_gradient){
                max_gradient=gradient[i];
                index_max_gradient=i; // cuidado porque se suma 1 por empezar de cero.
            }
                
        }
        // std::cout << "index max gradient: " << index_max_gradient << std::endl;
        // std::cout << "max gradient: " << max_gradient << std::endl;       

        // std::cout << "gradient_unit_vector X: " << gradient_unit_vector[index_max_gradient].x << std::endl;
        // std::cout << "gradient_unit_vector Y: " << gradient_unit_vector[index_max_gradient].y << std::endl;
        // std::cout << "gradient_unit_vector Z: " << gradient_unit_vector[index_max_gradient].z << std::endl;

        // Compute vector to choose the node
        Vec3f vectorNeighbours[1];
        int weight_gradient=1;
        vectorNeighbours[0].x=weight_gradient*gradient_unit_vector[index_max_gradient].x+attraction_unit_current[0].x;
        vectorNeighbours[0].y=weight_gradient*gradient_unit_vector[index_max_gradient].y+attraction_unit_current[0].y;
        vectorNeighbours[0].z=weight_gradient*gradient_unit_vector[index_max_gradient].z+attraction_unit_current[0].z;

        // WITHOUT CONSIDERING THE ATTRACTION OF THE GOAL
        // vectorNeighbours[0].x=weight_gradient*gradient_unit_vector[index_max_gradient].x;
        // vectorNeighbours[0].y=weight_gradient*gradient_unit_vector[index_max_gradient].y;
        // vectorNeighbours[0].z=weight_gradient*gradient_unit_vector[index_max_gradient].z;

        // std::cout << "vector X: " << vectorNeighbours[0].x << std::endl;
        // std::cout << "vector Y: " << vectorNeighbours[0].y << std::endl;    
        // std::cout << "vector Z: " << vectorNeighbours[0].z << std::endl;

        // Choose the neighbour nodes to explore
        float module_vectorNeighbours=0;
        module_vectorNeighbours = sqrt(pow(vectorNeighbours[0].x, 2) + pow(vectorNeighbours[0].y, 2) + pow(vectorNeighbours[0].z, 2));
        // std::cout << "module vectorNeighbours: " << module_vectorNeighbours << std::endl;

        // Angles of the vector
        float angle_h=0; // horizontal angle X-Y
        float angle_v=0; // vertical angle  
        float ang=0;
        angle_h=atan2(vectorNeighbours[0].y,vectorNeighbours[0].x);
        ang=vectorNeighbours[0].z/module_vectorNeighbours;
        // std::cout << "ang: " << ang << std::endl;
        angle_v=asin(ang);
        // angle_v=atan2(vectorNeighbours[0].z,vectorNeighbours[0].x);
        // std::cout << "angle h: " << angle_h << std::endl; 
        // std::cout << "angle v: " << angle_v << std::endl; 

        /********************************************************************************************************/
        // // CONSIDERING THE DIFFERENCE BETWEEN THE ANGLE OF THE GRADIENT AND THE ANGLE OF THE GOAL.
        // Angle of the gradient DESCOMENTAR
        float angle_gradient_h=0; // horizontal angle X-Y
        angle_gradient_h=atan2(gradient_unit_vector[index_max_gradient].y,gradient_unit_vector[index_max_gradient].x);
        // std::cout << "angle gradient h: " << angle_gradient_h << std::endl;

        // Angle of the attraction DESCOMENTAR
        float angle_attrac_h=0; // horizontal angle X-Y
        angle_attrac_h=atan2(attraction_unit_current[0].y,attraction_unit_current[0].x);
        // std::cout << "angle attrac h: " << angle_attrac_h << std::endl;

        unsigned int num_nodes;
        Vec3i node_choosenCoordinates[13];  //11
        int index_nodes[13]; //11

        if (angle_gradient_h != 0){
            if (abs(angle_attrac_h - angle_gradient_h) > (M_PI/2)){  //1.3962634: 80
            // It works with 9-10 and considering 75 degrees.
            // if (abs(angle_attrac_h - angle_gradient_h) > (1.3089)){  //1.308996939: 75
                num_nodes=13; //11
                // Vec3i node_choosenCoordinates[11]; 
                // int index_nodes[11];
                // std::cout << "111: " << std::endl;
            }
            else {
                num_nodes=11; //9
                // Vec3i node_choosenCoordinates[9]; 
                // int index_nodes[9];
                // std::cout << "ang: " << ang << std::endl;
                // angle_v=atan2(vectorNeighbours[0].z,vectorNeighbours[0].x);
                // std::cout << "222: " << std::endl;
            }
        }
        // If angle_gradient_h == 0
        else{
            num_nodes=13; //11
            // Vec3i node_choosenCoordinates[11]; 
            // int index_nodes[11];
            // std::cout << "333: " << std::endl;
        }
        /********************************************************************************************************/

        // TO CHANGE TO EXPLORE 9,11, 13, 15 or 17 NODES
        // Vec3i node_choosenCoordinates[11];
        // unsigned int num_nodes=11;
        // int index_nodes[11];
        
        // Function to compute the neighbours: Inputs --> angle_h and angle_v - Outputs --> List with neighbours
        int node_choosen;
        node_choosen=chooseNeighbours(angle_h, angle_v);
        // node_choosenCoordinates=chooseNeighbours(angle_h, angle_v);
        // std::cout << "node: " << node_choosen << std::endl;
        // std::cout << "coord_X: " << node_choosenCoordinates.x << std::endl;
        // std::cout << "coord_Y: " << node_choosenCoordinates.y << std::endl;
        // std::cout << "coord_Z: " << node_choosenCoordinates.z << std::endl;

        // Vec3i nodes_to_explore[9];
        // TODO Function to calculate the nodes to explore 
        // nodesToExplore(node_choosen);
        
        if (node_choosen == 0){
            //0,7,9,20,15,19,24,16,23
            index_nodes[0]=0;
            index_nodes[1]=7;
            index_nodes[2]=9;
            index_nodes[3]=20;
            index_nodes[4]=15;
            index_nodes[5]=19;
            index_nodes[6]=24;
            index_nodes[7]=16;
            index_nodes[8]=23;
            if (num_nodes == 10){
                index_nodes[9]=1;
            }
            else if (num_nodes==11){
                index_nodes[9]=2;
                index_nodes[10]=3;
            }
            else if(num_nodes==13){
                index_nodes[9]=2;
                index_nodes[10]=3;
                index_nodes[11]=10;
                index_nodes[12]=11;
            }
            else if(num_nodes==15){
                index_nodes[9]=2;
                index_nodes[10]=3;
                index_nodes[11]=11;
                index_nodes[12]=13;
                index_nodes[13]=10;
                index_nodes[14]=12;
            }
            else if (num_nodes==17){
                index_nodes[9]=13;
                index_nodes[10]=4;
                index_nodes[11]=11;
                index_nodes[12]=3;
                index_nodes[13]=2;
                index_nodes[14]=10;
                index_nodes[15]=5;
                index_nodes[16]=12;
            }
        }
        else if (node_choosen == 1){
            // 1,6,8,14,18,21,17,22,25
            index_nodes[0]=1;
            index_nodes[1]=6;
            index_nodes[2]=8;
            index_nodes[3]=14;
            index_nodes[4]=18;
            index_nodes[5]=21;
            index_nodes[6]=17;
            index_nodes[7]=22;
            index_nodes[8]=25;
            if (num_nodes == 10){
                index_nodes[9]=0;
            }
            else if (num_nodes==11){
                index_nodes[9]=2;
                index_nodes[10]=3;
            }
            else if(num_nodes==13){
                index_nodes[9]=2;
                index_nodes[10]=3;
                index_nodes[11]=12;
                index_nodes[12]=13;
            }
            else if(num_nodes==15){
                index_nodes[9]=2;
                index_nodes[10]=3;
                index_nodes[11]=12;
                index_nodes[12]=13;
                index_nodes[13]=10;
                index_nodes[14]=11;
            }
            else if (num_nodes==17){
                index_nodes[9]=13;
                index_nodes[10]=4;
                index_nodes[11]=11;
                index_nodes[12]=3;
                index_nodes[13]=2;
                index_nodes[14]=10;
                index_nodes[15]=5;
                index_nodes[16]=12;
            }
        }
        else if (node_choosen == 2){
            // 2,6,9,11,19,21,12,23,25
            index_nodes[0]=2;
            index_nodes[1]=6;
            index_nodes[2]=9;
            index_nodes[3]=11;
            index_nodes[4]=19;
            index_nodes[5]=21;
            index_nodes[6]=12;
            index_nodes[7]=23;
            index_nodes[8]=25;
            if (num_nodes == 10){
                index_nodes[9]=3;
            }
            else if (num_nodes==11){
                index_nodes[9]=0;
                index_nodes[10]=1;
            }
            else if(num_nodes==13){
                index_nodes[9]=0;
                index_nodes[10]=1;
                index_nodes[11]=17;
                index_nodes[12]=15;
            }
            else if(num_nodes==15){
                index_nodes[9]=0;
                index_nodes[10]=1;
                index_nodes[11]=17;
                index_nodes[12]=15;
                index_nodes[13]=16;
                index_nodes[14]=14;
            }
            else if (num_nodes==17){
                index_nodes[9]=15;
                index_nodes[10]=4;
                index_nodes[11]=14;
                index_nodes[12]=0;
                index_nodes[13]=1;
                index_nodes[14]=16;
                index_nodes[15]=5;
                index_nodes[16]=17;
            }
        }
        else if (node_choosen == 3){
            // 3,7,9,13,18,20,10,22,24
            index_nodes[0]=3;
            index_nodes[1]=7;
            index_nodes[2]=8;
            index_nodes[3]=13;
            index_nodes[4]=18;
            index_nodes[5]=20;
            index_nodes[6]=10;
            index_nodes[7]=22;
            index_nodes[8]=24;
            if (num_nodes == 10){
                index_nodes[9]=2;
            }
            else if (num_nodes==11){
                index_nodes[9]=0;
                index_nodes[10]=1;
            }
            else if(num_nodes==13){
                index_nodes[9]=0;
                index_nodes[10]=1;
                index_nodes[11]=16;
                index_nodes[12]=14;
            }
            else if(num_nodes==15){
                index_nodes[9]=0;
                index_nodes[10]=1;
                index_nodes[11]=16;
                index_nodes[12]=14;
                index_nodes[13]=17;
                index_nodes[14]=15;
            }
            else if (num_nodes==17){
                index_nodes[9]=15;
                index_nodes[10]=4;
                index_nodes[11]=14;
                index_nodes[12]=0;
                index_nodes[13]=1;
                index_nodes[14]=16;
                index_nodes[15]=5;
                index_nodes[16]=17;
            }
        }
        else if (node_choosen == 4){
            // 4,11,13,14,15,18,19,20,21
            index_nodes[0]=4;
            index_nodes[1]=11;
            index_nodes[2]=13;
            index_nodes[3]=14;
            index_nodes[4]=15;
            index_nodes[5]=18;
            index_nodes[6]=19;
            index_nodes[7]=20;
            index_nodes[8]=21;
            if (num_nodes == 10){
                index_nodes[9]=5;
            }
            else if (num_nodes==11){
                index_nodes[9]=0;
                index_nodes[10]=1;
            }
            else if (num_nodes==13){
                index_nodes[9]=0;
                index_nodes[10]=1;
                index_nodes[11]=2;
                index_nodes[12]=3;
            }
            else if(num_nodes==15){
                index_nodes[9]=0;
                index_nodes[10]=1;
                index_nodes[11]=2;
                index_nodes[12]=3;
                index_nodes[13]=6;
                index_nodes[14]=7;
            }
            else if (num_nodes==17){
                index_nodes[9]=7;
                index_nodes[10]=0;
                index_nodes[11]=9;
                index_nodes[12]=3;
                index_nodes[13]=2;
                index_nodes[14]=8;
                index_nodes[15]=1;
                index_nodes[16]=6;
            }
        }
        else if (node_choosen == 5){
            // 5,10,12,16,17,22,23,24,25
            index_nodes[0]=5;
            index_nodes[1]=10;
            index_nodes[2]=12;
            index_nodes[3]=16;
            index_nodes[4]=17;
            index_nodes[5]=22;
            index_nodes[6]=23;
            index_nodes[7]=24;
            index_nodes[8]=25;
            if (num_nodes == 10){
                index_nodes[9]=4;
            }
            else if (num_nodes==11){
                index_nodes[9]=0;
                index_nodes[10]=1;
            }
            else if (num_nodes==13){
                index_nodes[9]=0;
                index_nodes[10]=1;
                index_nodes[11]=2;
                index_nodes[12]=3;
            }
            else if(num_nodes==15){
                index_nodes[9]=0;
                index_nodes[10]=1;
                index_nodes[11]=2;
                index_nodes[12]=3;
                index_nodes[13]=8;
                index_nodes[14]=9;
            }
            else if (num_nodes==17){
                index_nodes[9]=7;
                index_nodes[10]=0;
                index_nodes[11]=9;
                index_nodes[12]=3;
                index_nodes[13]=2;
                index_nodes[14]=8;
                index_nodes[15]=1;
                index_nodes[16]=6;
            }
        }
        else if (node_choosen == 6){
            // 6,1,2,11,14,21,12,17,25
            index_nodes[0]=6;
            index_nodes[1]=1;
            index_nodes[2]=2;
            index_nodes[3]=11;
            index_nodes[4]=14;
            index_nodes[5]=21;
            index_nodes[6]=12;
            index_nodes[7]=17;
            index_nodes[8]=25;
            if (num_nodes == 10){
                index_nodes[9]=7;
            }
            else if (num_nodes==11){
                index_nodes[9]=8;
                index_nodes[10]=9;
            }
            else if(num_nodes==13){
                index_nodes[9]=8;
                index_nodes[10]=9;
                index_nodes[11]=22;
                index_nodes[12]=19;
            }
            else if(num_nodes==15){
                index_nodes[9]=4;
                index_nodes[10]=5;
                index_nodes[11]=22;
                index_nodes[12]=19;
                index_nodes[13]=18;
                index_nodes[14]=23;
            }
            else if (num_nodes==17){
                index_nodes[9]=9;
                index_nodes[10]=8;
                index_nodes[11]=18;
                index_nodes[12]=19;
                index_nodes[13]=22;
                index_nodes[14]=23;
                index_nodes[15]=4;
                index_nodes[16]=5;
            }
        }
        else if (node_choosen == 7){
            // 7,0,3,13,20,15,10,24,17
            index_nodes[0]=7;
            index_nodes[1]=0;
            index_nodes[2]=3;
            index_nodes[3]=13;
            index_nodes[4]=20;
            index_nodes[5]=15;
            index_nodes[6]=10;
            index_nodes[7]=24;
            index_nodes[8]=16;
            if (num_nodes == 10){
                index_nodes[9]=6;
            }
            else if (num_nodes==11){
                index_nodes[9]=8;
                index_nodes[10]=9;
            }
            else if(num_nodes==13){
                index_nodes[9]=8;
                index_nodes[10]=9;
                index_nodes[11]=23;
                index_nodes[12]=18;
            }
            else if(num_nodes==15){
                index_nodes[9]=8;
                index_nodes[10]=9;
                index_nodes[11]=23;
                index_nodes[12]=18;
                index_nodes[13]=19;
                index_nodes[14]=22;
            }
            else if (num_nodes==17){
                index_nodes[9]=8;
                index_nodes[10]=9;
                index_nodes[11]=18;
                index_nodes[12]=19;
                index_nodes[13]=22;
                index_nodes[14]=23;
                index_nodes[15]=4;
                index_nodes[16]=5;
            }
        }
        else if (node_choosen == 8){
            // 8,1,3,13,18,14,10,22,17
            index_nodes[0]=8;
            index_nodes[1]=1;
            index_nodes[2]=3;
            index_nodes[3]=13;
            index_nodes[4]=18;
            index_nodes[5]=14;
            index_nodes[6]=10;
            index_nodes[7]=22;
            index_nodes[8]=17;
            if (num_nodes == 10){
                index_nodes[9]=9;
            }
            else if (num_nodes==11){
                index_nodes[9]=6;
                index_nodes[10]=7;
            }
            else if(num_nodes==13){
                index_nodes[9]=6;
                index_nodes[10]=7;
                index_nodes[11]=24;
                index_nodes[12]=21;
            }
            else if(num_nodes==15){
                index_nodes[9]=6;
                index_nodes[10]=7;
                index_nodes[11]=24;
                index_nodes[12]=21;
                index_nodes[13]=20;
                index_nodes[14]=25;
            }
            else if (num_nodes==17){
                index_nodes[9]=6;
                index_nodes[10]=7;
                index_nodes[11]=20;
                index_nodes[12]=21;
                index_nodes[13]=24;
                index_nodes[14]=25;
                index_nodes[15]=4;
                index_nodes[16]=5;
            }
        }
        else if (node_choosen == 9){
            // 9,0,2,15,19,11,16,23,12
            index_nodes[0]=9;
            index_nodes[1]=0;
            index_nodes[2]=2;
            index_nodes[3]=15;
            index_nodes[4]=19;
            index_nodes[5]=11;
            index_nodes[6]=16;
            index_nodes[7]=23;
            index_nodes[8]=12;
            if (num_nodes == 10){
                index_nodes[9]=8;
            }
            else if (num_nodes==11){
                index_nodes[9]=6;
                index_nodes[10]=7;
            }
            else if(num_nodes==13){
                index_nodes[9]=6;
                index_nodes[10]=7;
                index_nodes[11]=20;
                index_nodes[12]=25;
            }
            else if(num_nodes==15){
                index_nodes[9]=6;
                index_nodes[10]=7;
                index_nodes[11]=20;
                index_nodes[12]=25;
                index_nodes[13]=24;
                index_nodes[14]=21;
            }
            else if (num_nodes==17){
                index_nodes[9]=6;
                index_nodes[10]=7;
                index_nodes[11]=20;
                index_nodes[12]=21;
                index_nodes[13]=24;
                index_nodes[14]=25;
                index_nodes[15]=4;
                index_nodes[16]=5;
            }
        }
        else if (node_choosen == 10){
            // 10,24,22,7,3,8,16,5,17
            index_nodes[0]=10;
            index_nodes[1]=24;
            index_nodes[2]=22;
            index_nodes[3]=7;
            index_nodes[4]=3;
            index_nodes[5]=8;
            index_nodes[6]=16;
            index_nodes[7]=5;
            index_nodes[8]=17;
            if (num_nodes == 10){
                index_nodes[9]=11;
            }
            else if (num_nodes==11){
                index_nodes[9]=0;
                index_nodes[10]=1;
            }
            else if(num_nodes==13){
                index_nodes[9]=0;
                index_nodes[10]=1;
                index_nodes[11]=18;
                index_nodes[12]=23;
            }
            else if(num_nodes==15){
                index_nodes[9]=0;
                index_nodes[10]=1;
                index_nodes[11]=18;
                index_nodes[12]=23;
                index_nodes[13]=20;
                index_nodes[14]=25;
            }
            else if (num_nodes==17){
                index_nodes[9]=23;
                index_nodes[10]=12;
                index_nodes[11]=25;
                index_nodes[12]=0;
                index_nodes[13]=1;
                index_nodes[14]=20;
                index_nodes[15]=13;
                index_nodes[16]=18;
            }
        }
        else if (node_choosen == 11){
            // 11,19,21,9,2,6,15,4,14
            index_nodes[0]=11;
            index_nodes[1]=19;
            index_nodes[2]=21;
            index_nodes[3]=9;
            index_nodes[4]=2;
            index_nodes[5]=6;
            index_nodes[6]=15;
            index_nodes[7]=4;
            index_nodes[8]=14;
            if (num_nodes == 10){
                index_nodes[9]=10;
            }
            else if (num_nodes==11){
                index_nodes[9]=0;
                index_nodes[10]=1;
            }
            else if(num_nodes==13){
                index_nodes[9]=0;
                index_nodes[10]=1;
                index_nodes[11]=25;
                index_nodes[12]=20;
            }
            else if(num_nodes==15){
                index_nodes[9]=0;
                index_nodes[10]=1;
                index_nodes[11]=25;
                index_nodes[12]=20;
                index_nodes[13]=18;
                index_nodes[14]=23;
            }
            else if (num_nodes==17){
                index_nodes[9]=20;
                index_nodes[10]=13;
                index_nodes[11]=18;
                index_nodes[12]=0;
                index_nodes[13]=1;
                index_nodes[14]=23;
                index_nodes[15]=12;
                index_nodes[16]=25;
            }
        }
        else if (node_choosen == 12){
            // 12,23,25,9,2,6,16,5,17
            index_nodes[0]=12;
            index_nodes[1]=23;
            index_nodes[2]=25;
            index_nodes[3]=9;
            index_nodes[4]=2;
            index_nodes[5]=6;
            index_nodes[6]=16;
            index_nodes[7]=5;
            index_nodes[8]=17;
            if (num_nodes == 10){
                index_nodes[9]=13;
            }
            else if (num_nodes==11){
                index_nodes[9]=0;
                index_nodes[10]=1;
            }
            else if(num_nodes==13){
                index_nodes[9]=0;
                index_nodes[10]=1;
                index_nodes[11]=22;
                index_nodes[12]=19;
            }
            else if(num_nodes==15){
                index_nodes[9]=0;
                index_nodes[10]=1;
                index_nodes[11]=22;
                index_nodes[12]=19;
                index_nodes[13]=24;
                index_nodes[14]=21;
            }
            else if (num_nodes==17){
                index_nodes[9]=24;
                index_nodes[10]=10;
                index_nodes[11]=22;
                index_nodes[12]=0;
                index_nodes[13]=1;
                index_nodes[14]=19;
                index_nodes[15]=11;
                index_nodes[16]=21;
            }
        }
        else if (node_choosen == 13){
            // 13,20,18,7,3,8,15,4,14
            index_nodes[0]=13;
            index_nodes[1]=20;
            index_nodes[2]=18;
            index_nodes[3]=7;
            index_nodes[4]=3;
            index_nodes[5]=8;
            index_nodes[6]=15;
            index_nodes[7]=4;
            index_nodes[8]=14;
            if (num_nodes == 10){
                index_nodes[9]=12;
            }
            else if (num_nodes==11){
                index_nodes[9]=0;
                index_nodes[10]=1;
            }
            else if(num_nodes==13){
                index_nodes[9]=0;
                index_nodes[10]=1;
                index_nodes[11]=22;
                index_nodes[12]=19;
            }
            else if(num_nodes==15){
                index_nodes[9]=0;
                index_nodes[10]=1;
                index_nodes[11]=22;
                index_nodes[12]=19;
                index_nodes[13]=24;
                index_nodes[14]=21;
            }
            else if (num_nodes==17){
                index_nodes[9]=19;
                index_nodes[10]=11;
                index_nodes[11]=21;
                index_nodes[12]=0;
                index_nodes[13]=1;
                index_nodes[14]=24;
                index_nodes[15]=10;
                index_nodes[16]=22;
            }
        }
        else if (node_choosen == 14){
            // 14, 4, 11, 13, 18, 21, 1, 6, 8
            index_nodes[0]=14;
            index_nodes[1]=4;
            index_nodes[2]=11;
            index_nodes[3]=13;
            index_nodes[4]=18;
            index_nodes[5]=21;
            index_nodes[6]=1;
            index_nodes[7]=6;
            index_nodes[8]=8;
            if (num_nodes == 10){
                index_nodes[9]=16;
            }
            else if (num_nodes==11){
                index_nodes[9]=2;
                index_nodes[10]=3;
            }
            else if(num_nodes==13){
                index_nodes[9]=2;
                index_nodes[10]=3;
                index_nodes[11]=22;
                index_nodes[12]=19;
            }
            else if(num_nodes==15){
                index_nodes[9]=2;
                index_nodes[10]=3;
                index_nodes[11]=22;
                index_nodes[12]=19;
                index_nodes[13]=25;
                index_nodes[14]=20;
            }
            else if (num_nodes==17){
                index_nodes[9]=20;
                index_nodes[10]=15;
                index_nodes[11]=19;
                index_nodes[12]=2;
                index_nodes[13]=3;
                index_nodes[14]=22;
                index_nodes[15]=17;
                index_nodes[16]=25;
            }
        }
        else if (node_choosen == 15){
            // 15,19,20,7,0,9,13,4,11
            index_nodes[0]=15;
            index_nodes[1]=19;
            index_nodes[2]=20;
            index_nodes[3]=7;
            index_nodes[4]=0;
            index_nodes[5]=9;
            index_nodes[6]=13;
            index_nodes[7]=4;
            index_nodes[8]=11;
            if (num_nodes == 10){
                index_nodes[9]=17;
            }
            else if (num_nodes==11){
                index_nodes[9]=2;
                index_nodes[10]=3;
            }
            else if(num_nodes==13){
                index_nodes[9]=2;
                index_nodes[10]=3;
                index_nodes[11]=24;
                index_nodes[12]=21;
            }
            else if(num_nodes==15){
                index_nodes[9]=2;
                index_nodes[10]=3;
                index_nodes[11]=24;
                index_nodes[12]=21;
                index_nodes[13]=23;
                index_nodes[14]=18;
            }
            else if (num_nodes==17){
                index_nodes[9]=18;
                index_nodes[10]=14;
                index_nodes[11]=21;
                index_nodes[12]=2;
                index_nodes[13]=3;
                index_nodes[14]=24;
                index_nodes[15]=16;
                index_nodes[16]=23;
            }
        }
        else if (node_choosen == 16){
            // 16,24,23,7,0,9,10,5,12
            index_nodes[0]=16;
            index_nodes[1]=24;
            index_nodes[2]=23;
            index_nodes[3]=7;
            index_nodes[4]=0;
            index_nodes[5]=9;
            index_nodes[6]=10;
            index_nodes[7]=5;
            index_nodes[8]=12;
            if (num_nodes == 10){
                index_nodes[9]=14;
            }
            else if (num_nodes==11){
                index_nodes[9]=2;
                index_nodes[10]=3;
            }
            else if(num_nodes==13){
                index_nodes[9]=2;
                index_nodes[10]=3;
                index_nodes[11]=22;
                index_nodes[12]=19;
            }
            else if(num_nodes==15){
                index_nodes[9]=2;
                index_nodes[10]=3;
                index_nodes[11]=22;
                index_nodes[12]=19;
                index_nodes[13]=25;
                index_nodes[14]=20;
            }
            else if (num_nodes==17){
                index_nodes[9]=22;
                index_nodes[10]=17;
                index_nodes[11]=25;
                index_nodes[12]=2;
                index_nodes[13]=3;
                index_nodes[14]=20;
                index_nodes[15]=15;
                index_nodes[16]=19;
            }
        }
        else if (node_choosen == 17){
            // 17,22,25,8,1,6,10,5,12
            index_nodes[0]=17;
            index_nodes[1]=22;
            index_nodes[2]=25;
            index_nodes[3]=8;
            index_nodes[4]=1;
            index_nodes[5]=6;
            index_nodes[6]=10;
            index_nodes[7]=5;
            index_nodes[8]=12;
            if (num_nodes == 10){
                index_nodes[9]=15;
            }
            else if (num_nodes==11){
                index_nodes[9]=2;
                index_nodes[10]=3;
            }
            else if(num_nodes==13){
                index_nodes[9]=2;
                index_nodes[10]=3;
                index_nodes[11]=24;
                index_nodes[12]=21;
            }
            else if(num_nodes==15){
                index_nodes[9]=2;
                index_nodes[10]=3;
                index_nodes[11]=24;
                index_nodes[12]=21;
                index_nodes[13]=23;
                index_nodes[14]=18;
            }
            else if (num_nodes==17){
                index_nodes[9]=24;
                index_nodes[10]=16;
                index_nodes[11]=23;
                index_nodes[12]=2;
                index_nodes[13]=3;
                index_nodes[14]=18;
                index_nodes[15]=14;
                index_nodes[16]=21;
            }
        }
        else if (node_choosen == 18){
            // 18,13,15,3,8,1,20,4,21
            index_nodes[0]=18;
            index_nodes[1]=13;
            index_nodes[2]=14;
            index_nodes[3]=3;
            index_nodes[4]=8;
            index_nodes[5]=1;
            index_nodes[6]=20;
            index_nodes[7]=4;
            index_nodes[8]=21;
            if (num_nodes == 10){
                index_nodes[9]=23;
            }
            else if (num_nodes==11){
                index_nodes[9]=6;
                index_nodes[10]=7;
            }
            else if(num_nodes==13){
                index_nodes[9]=6;
                index_nodes[10]=7;
                index_nodes[11]=10;
                index_nodes[12]=11;
            }
            else if(num_nodes==15){
                index_nodes[9]=6;
                index_nodes[10]=7;
                index_nodes[11]=10;
                index_nodes[12]=11;
                index_nodes[13]=17;
                index_nodes[14]=15;
            }
            else if (num_nodes==17){
                index_nodes[9]=15;
                index_nodes[10]=11;
                index_nodes[11]=19;
                index_nodes[12]=6;
                index_nodes[13]=7;
                index_nodes[14]=10;
                index_nodes[15]=22;
                index_nodes[16]=17;
            }
        }
        else if (node_choosen == 19){
            // 19,15,11,20,4,21,0,9,2
            index_nodes[0]=19;
            index_nodes[1]=15;
            index_nodes[2]=11;
            index_nodes[3]=20;
            index_nodes[4]=4;
            index_nodes[5]=21;
            index_nodes[6]=0;
            index_nodes[7]=9;
            index_nodes[8]=2;
            if (num_nodes == 10){
                index_nodes[9]=22;
            }
            else if (num_nodes==11){
                index_nodes[9]=6;
                index_nodes[10]=7;
            }
            else if(num_nodes==13){
                index_nodes[9]=6;
                index_nodes[10]=7;
                index_nodes[11]=16;
                index_nodes[12]=14;
            }
            else if(num_nodes==15){
                index_nodes[9]=6;
                index_nodes[10]=7;
                index_nodes[11]=16;
                index_nodes[12]=14;
                index_nodes[13]=12;
                index_nodes[14]=13;
            }
            else if (num_nodes==17){
                index_nodes[9]=13;
                index_nodes[10]=14;
                index_nodes[11]=18;
                index_nodes[12]=6;
                index_nodes[13]=7;
                index_nodes[14]=12;
                index_nodes[15]=16;
                index_nodes[16]=23;
            }
        }
        else if (node_choosen == 20){
            // 20,13,15,18,4,19,7,0,3
            index_nodes[0]=20;
            index_nodes[1]=13;
            index_nodes[2]=15;
            index_nodes[3]=18;
            index_nodes[4]=4;
            index_nodes[5]=19;
            index_nodes[6]=7;
            index_nodes[7]=0;
            index_nodes[8]=3;
            if (num_nodes == 10){
                index_nodes[9]=25;
            }
            else if (num_nodes==11){
                index_nodes[9]=8;
                index_nodes[10]=9;
            }
            else if(num_nodes==13){
                index_nodes[9]=8;
                index_nodes[10]=9;
                index_nodes[11]=10;
                index_nodes[12]=11;
            }
            else if(num_nodes==15){
                index_nodes[9]=8;
                index_nodes[10]=9;
                index_nodes[11]=10;
                index_nodes[12]=11;
                index_nodes[13]=16;
                index_nodes[14]=14;
            }
            else if (num_nodes==17){
                index_nodes[9]=11;
                index_nodes[10]=14;
                index_nodes[11]=21;
                index_nodes[12]=8;
                index_nodes[13]=9;
                index_nodes[14]=10;
                index_nodes[15]=24;
                index_nodes[16]=16;
            }
        }
        else if (node_choosen == 21){
            // 21,11,14,18,4,19,1,6,2
            index_nodes[0]=21;
            index_nodes[1]=11;
            index_nodes[2]=14;
            index_nodes[3]=18;
            index_nodes[4]=4;
            index_nodes[5]=19;
            index_nodes[6]=1;
            index_nodes[7]=6;
            index_nodes[8]=2;
            if (num_nodes == 10){
                index_nodes[9]=24;
            }
            else if (num_nodes==11){
                index_nodes[9]=8;
                index_nodes[10]=9;
            }
            else if(num_nodes==13){
                index_nodes[9]=8;
                index_nodes[10]=9;
                index_nodes[11]=17;
                index_nodes[12]=15;
            }
            else if(num_nodes==15){
                index_nodes[9]=8;
                index_nodes[10]=9;
                index_nodes[11]=17;
                index_nodes[12]=15;
                index_nodes[13]=12;
                index_nodes[14]=13;
            }
            else if (num_nodes==17){
                index_nodes[9]=13;
                index_nodes[10]=15;
                index_nodes[11]=20;
                index_nodes[12]=8;
                index_nodes[13]=9;
                index_nodes[14]=12;
                index_nodes[15]=17;
                index_nodes[16]=25;
            }
        }
        else if (node_choosen == 22){
            // 22,10,17,24,5,25,3,8,1
            index_nodes[0]=22;
            index_nodes[1]=10;
            index_nodes[2]=17;
            index_nodes[3]=24;
            index_nodes[4]=5;
            index_nodes[5]=25;
            index_nodes[6]=3;
            index_nodes[7]=8;
            index_nodes[8]=1;
            if (num_nodes == 10){
                index_nodes[9]=19;
            }
            else if (num_nodes==11){
                index_nodes[9]=6;
                index_nodes[10]=7;
            }
            else if(num_nodes==13){
                index_nodes[9]=6;
                index_nodes[10]=7;
                index_nodes[11]=16;
                index_nodes[12]=14;
            }
            else if(num_nodes==15){
                index_nodes[9]=6;
                index_nodes[10]=7;
                index_nodes[11]=16;
                index_nodes[12]=14;
                index_nodes[13]=12;
                index_nodes[14]=13;
            }
            else if (num_nodes==17){
                index_nodes[9]=16;
                index_nodes[10]=12;
                index_nodes[11]=23;
                index_nodes[12]=6;
                index_nodes[13]=7;
                index_nodes[14]=13;
                index_nodes[15]=14;
                index_nodes[16]=18;
            }
        }
        else if (node_choosen == 23){
            // 23,16,12,24,5,25,0,9,2
            index_nodes[0]=23;
            index_nodes[1]=16;
            index_nodes[2]=12;
            index_nodes[3]=24;
            index_nodes[4]=5;
            index_nodes[5]=25;
            index_nodes[6]=0;
            index_nodes[7]=9;
            index_nodes[8]=2;
            if (num_nodes == 10){
                index_nodes[9]=18;
            }
            else if (num_nodes==11){
                index_nodes[9]=6;
                index_nodes[10]=7;
            }
            else if(num_nodes==13){
                index_nodes[9]=6;
                index_nodes[10]=7;
                index_nodes[11]=10;
                index_nodes[12]=11;
            }
            else if(num_nodes==15){
                index_nodes[9]=6;
                index_nodes[10]=7;
                index_nodes[11]=10;
                index_nodes[12]=11;
                index_nodes[13]=17;
                index_nodes[14]=15;
            }
            else if (num_nodes==17){
                index_nodes[9]=10;
                index_nodes[10]=17;
                index_nodes[11]=22;
                index_nodes[12]=6;
                index_nodes[13]=7;
                index_nodes[14]=11;
                index_nodes[15]=19;
                index_nodes[16]=15;
            }
        }
        else if (node_choosen == 24){
            // 24,16,10,22,5,23,3,8,1
            index_nodes[0]=24;
            index_nodes[1]=16;
            index_nodes[2]=10;
            index_nodes[3]=22;
            index_nodes[4]=5;
            index_nodes[5]=23;
            index_nodes[6]=3;
            index_nodes[7]=7;
            index_nodes[8]=0;
            if (num_nodes == 10){
                index_nodes[9]=21;
            }
            else if (num_nodes==11){
                index_nodes[9]=8;
                index_nodes[10]=9;
            }
            else if(num_nodes==13){
                index_nodes[9]=8;
                index_nodes[10]=9;
                index_nodes[11]=17;
                index_nodes[12]=15;
            }
            else if(num_nodes==15){
                index_nodes[9]=8;
                index_nodes[10]=9;
                index_nodes[11]=17;
                index_nodes[12]=15;
                index_nodes[13]=12;
                index_nodes[14]=13;
            }
            else if (num_nodes==17){
                index_nodes[9]=12;
                index_nodes[10]=17;
                index_nodes[11]=25;
                index_nodes[12]=8;
                index_nodes[13]=9;
                index_nodes[14]=13;
                index_nodes[15]=15;
                index_nodes[16]=20;
            }
        }
        else if (node_choosen == 25){
            // 25,17,12,22,5,23,1,6,2
            index_nodes[0]=25;
            index_nodes[1]=17;
            index_nodes[2]=12;
            index_nodes[3]=22;
            index_nodes[4]=5;
            index_nodes[5]=23;
            index_nodes[6]=1;
            index_nodes[7]=6;
            index_nodes[8]=2;
            if (num_nodes == 10){
                index_nodes[9]=20;
            }
            else if (num_nodes==11){
                index_nodes[9]=8;
                index_nodes[10]=9;
            }
            else if(num_nodes==13){
                index_nodes[9]=8;
                index_nodes[10]=9;
                index_nodes[11]=10;
                index_nodes[12]=11;
            }
            else if(num_nodes==15){
                index_nodes[9]=8;
                index_nodes[10]=9;
                index_nodes[11]=10;
                index_nodes[12]=11;
                index_nodes[13]=16;
                index_nodes[14]=14;
            }
            else if (num_nodes==17){
                index_nodes[9]=10;
                index_nodes[10]=16;
                index_nodes[11]=24;
                index_nodes[12]=8;
                index_nodes[13]=9;
                index_nodes[14]=11;
                index_nodes[15]=14;
                index_nodes[16]=21;
            }
        }        
        for (unsigned int i = 0; i < num_nodes; ++i) {
            node_choosenCoordinates[i]=direction[index_nodes[i]];
        }

        // direction.size() should be substituted by the number of neighbours to explore
        // 9 is the size of node_choosenCoordinates
        // for (unsigned int i = 0; i < 9; ++i) { // when we consider the gradient to choose explored nodes
        // for (unsigned int i = 0; i < direction.size(); ++i) {
        for (unsigned int i = 0; i < num_nodes; ++i) { // when we consider the gradient to choose explored nodes

            // Vec3i newCoordinates = _current->coordinates + node_choosenCoordinates[i]; // when we consider the gradient to choose explored nodes
            Vec3i newCoordinates = _current->coordinates + direction[i];
            Node *successor = discrete_world_.getNodePtr(newCoordinates);

            if ( successor == nullptr || 
                 successor->isInClosedList || 
                 successor->occuppied ) 
                continue;
    
            unsigned int totalCost = computeG(_current, successor, index_nodes[i], direction.size());

            if (! successor->isInOpenList ) { 

                // successor->parent = _current;
                // // successor->G = computeG(_current, successor, i, direction.size());
                // successor->G = computeG(_current, successor, gradient[i], index_nodes[i], direction.size());
                // successor->H = heuristic(successor->coordinates, _target);
                // // atraction[i]=getVectorPull(successor->coordinates, _target);
                // // std::cout << "atraction X: " << atraction[i].x << std::endl;
                // // std::cout << "atraction Y: " << atraction[i].y << std::endl;
                // // std::cout << "atraction Z: " << atraction[i].z << std::endl;
                // successor->gplush = successor->G + successor->H;
                // successor->isInOpenList = true;
                // _index_by_pos.insert(successor);

                successor->parent = _current;
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, _target);
                successor->gplush = successor->G + successor->H;
                successor->isInOpenList = true;
                _index_by_pos.insert(successor);
            }
            else if (totalCost < successor->G) {
                successor->parent = _current;
                successor->G = totalCost;
                successor->gplush = successor->G + successor->H;
                auto found = _index_by_pos.find(successor->world_index);
                _index_by_pos.erase(found);
                _index_by_pos.insert(successor);
            }

            // std::cout << "target X: " << _target.x << std::endl;
            // std::cout << "target Y: " << _target.y << std::endl;
            // std::cout << "target Z: " << _target.z << std::endl;
        }

        // for (unsigned int i = 0; i < direction.size(); ++i) {

        //     Vec3i newCoordinates = _current->coordinates + direction[i];
        //     Node *successor = discrete_world_.getNodePtr(newCoordinates);

        //     if ( successor == nullptr || 
        //          successor->isInClosedList || 
        //          successor->occuppied ) 
        //         continue;
    
        //     if (! successor->isInOpenList ) { 

        //         successor->parent = _current;
        //         successor->G = computeG(_current, successor, i, direction.size());
        //         successor->H = heuristic(successor->coordinates, _target);
        //         successor->gplush = successor->G + successor->H;
        //         successor->isInOpenList = true;
        //         _index_by_pos.insert(successor);
        //     }
        // }
    }

    int AStarEDF::chooseNeighbours(float angh, float angv){
    // Vec3i ThetaStar::chooseNeighbours(float angh, float angv){
        //choose_node.m (folder Gradients)
        
        // std::cout << "angh: " << angh << std::endl; 
        // std::cout << "angv: " << angv << std::endl; 
        int node=0;
        // Vec3i nodeCoordinates;

        // // Modules
        // float mod[26];
        // float aux;
        // for (unsigned int i = 0; i < direction.size(); ++i)
        // {
        //     aux=0;
        //     mod[i]= (i < 6 ? dist_scale_factor_ : (i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
        //     aux=mod[i]/100;
        //     mod[i]=aux;
        // }
        // // std::cout << "mod: " << mod[0] << std::endl;
        
        // // Angles
        // float angles_h[26], angles_v[26];
        float diff_h[26],diff_v[26];
        float ang_aux; //, ang_aux_v;

        for (unsigned int i = 0; i < direction.size(); ++i) {

            // Vec3i newCoordinates = direction[i];

            // angles_h[i]=atan2(newCoordinates.y,newCoordinates.x);
            // angles_v[i]=asin((newCoordinates.z/mod[i]));
            // dif_h
            if (angh < 0){
                if (angles_h[i] < 0){
                    diff_h[i]=abs(angh-angles_h[i]); 
                }
                else if (angles_h[i] == 0){
                    diff_h[i]=abs(angh);
                }
                else if (angles_h[i] == M_PI){
                    diff_h[i]=abs(M_PI-angh);
                }
                else {
                    ang_aux=2*M_PI+angh;
                    diff_h[i]=abs(ang_aux-angles_h[i]);
                }
            }
            else {
                if (angles_h[i] < 0){
                    ang_aux=2*M_PI+angles_h[i];
                    diff_h[i]=abs(angh-ang_aux);
                }
                else {
                    diff_h[i]=abs(angh-angles_h[i]); 
                }
                
            }
            // //dif_v
            diff_v[i]=angv-angles_v[i];

            // std::cout << "coord_X: " << newCoordinates.x << std::endl;
            // std::cout << "coord_Y: " << newCoordinates.y << std::endl;
            // std::cout << "indice: " << i << std::endl;
            // std::cout << "angles_h: " << angles_h[i] << std::endl;
            // std::cout << "angles_v: " << angles_v[i] << std::endl;
            // std::cout << "diff_h: " << diff_h[i] << std::endl;
            // std::cout << "diff_v: " << diff_v[i] << std::endl;
        }        
        // std::cout << "diff_v: " << diff_v[0] << std::endl;
        

        // Compute the minimum angles_h
        float min_diff_h=M_PI;
        int index_min_angle_h=0;
        for (unsigned int i = 0; i < direction.size(); ++i) {
            if (diff_h[i] < min_diff_h){
                min_diff_h=diff_h[i];
                index_min_angle_h=i; // cuidado porque se suma 1 por empezar de cero.
            }
        }
        // std::cout << "min_diff_h: " << min_diff_h << std::endl;
        // std::cout << "indicie elegido: " << index_min_angle_h << std::endl;

        if( index_min_angle_h == 0 ){
            // std::cout << "node: " << node << std::endl;
            if ((diff_v[index_min_angle_h] < (angles_v[11]/2)) && (diff_v[index_min_angle_h] > (-angles_v[11]/2))){
                node=0; // the same as index_min_angle_h
            }
            else if (diff_v[index_min_angle_h] > (angles_v[11]/2)){
                node=15;
            }
            else if (diff_v[index_min_angle_h] < (-angles_v[11]/2)){
                node=16;
            }
        }else if( index_min_angle_h == 1 ){
            // std::cout << "node: " << node << std::endl; 
            if ((diff_v[index_min_angle_h] < (angles_v[11]/2)) && (diff_v[index_min_angle_h] > (-angles_v[11]/2))){
                node=1; // the same as index_min_angle_h
            }
            else if (diff_v[index_min_angle_h] > (angles_v[11]/2)){
                node=14;
            }
            else if (diff_v[index_min_angle_h] < (-angles_v[11]/2)){
                node=17;
            }
        }else if( index_min_angle_h == 2 ){
            // std::cout << "node: " << node << std::endl; 
            if ((diff_v[index_min_angle_h] < (angles_v[11]/2)) && (diff_v[index_min_angle_h] > (-angles_v[11]/2))){
                node=2; // the same as index_min_angle_h
            }
            else if ((diff_v[index_min_angle_h] > (angles_v[11]/2)) && (diff_v[index_min_angle_h] < (3*(angles_v[11]/2)))){
                node=11;
            }
            else if ((diff_v[index_min_angle_h] < (-(angles_v[11]/2))) && (diff_v[index_min_angle_h] > (-3*(angles_v[11]/2)))){
                node=12;
            }
            else if (diff_v[index_min_angle_h] > (3*(angles_v[11]/2))){
                node=4;
            }
            else if (diff_v[index_min_angle_h] < (-3*(-angles_v[11]/2))){
                node=5;
            }
        }else if( index_min_angle_h == 3 ){
            // std::cout << "node: " << node << std::endl; 
            if ((diff_v[index_min_angle_h] < (angles_v[11]/2)) && (diff_v[index_min_angle_h] > (-angles_v[11]/2))){
                node=3; // the same as index_min_angle_h
            }
            else if (diff_v[index_min_angle_h] > (angles_v[11]/2)){
                node=13;
            }
            else if (diff_v[index_min_angle_h] < (-angles_v[11]/2)){
                node=10;
            } 
        }else if( index_min_angle_h == 6 ){
            // std::cout << "node: " << node << std::endl; 
            if ((diff_v[index_min_angle_h] < (angles_v[19]/2)) && (diff_v[index_min_angle_h] > (-angles_v[19]/2))){
                node=6; // the same as index_min_angle_h
            }
            else if (diff_v[index_min_angle_h] > (angles_v[19]/2)){
                node=21;
            }
            else if (diff_v[index_min_angle_h] < (-angles_v[19]/2)){
                node=25;
            }
        }else if( index_min_angle_h == 7 ){
            // std::cout << "node: " << node << std::endl; 
            if ((diff_v[index_min_angle_h] < (angles_v[19]/2)) && (diff_v[index_min_angle_h] > (-angles_v[19]/2))){
                node=7; // the same as index_min_angle_h
            }
            else if (diff_v[index_min_angle_h] > (angles_v[19]/2)){
                node=20;
            }
            else if (diff_v[index_min_angle_h] < (-angles_v[19]/2)){
                node=24;
            }
        }else if( index_min_angle_h == 8 ){
            if ((diff_v[index_min_angle_h] < (angles_v[19]/2)) && (diff_v[index_min_angle_h] > (-angles_v[19]/2))){
                node=8; // the same as index_min_angle_h
            }
            else if (diff_v[index_min_angle_h] > (angles_v[19]/2)){
                node=18;
            }
            else if (diff_v[index_min_angle_h] < (-angles_v[19]/2)){
                node=22;
            }
            // std::cout << "node: " << node << std::endl; 
        }else if( index_min_angle_h == 9 ){
            // std::cout << "node: " << node << std::endl; 
            if ((diff_v[index_min_angle_h] < (angles_v[19]/2)) && (diff_v[index_min_angle_h] > (-angles_v[19]/2))){
                node=9; // the same as index_min_angle_h
            }
            else if (diff_v[index_min_angle_h] > (angles_v[19]/2)){
                node=19;
            }
            else if (diff_v[index_min_angle_h] < (-angles_v[19]/2)){
                node=23;
            }
        }
        // std::cout << "node: " << node << std::endl;
        return (node);
        // nodeCoordinates=direction[node];
        // return(nodeCoordinates);
    }

    void AStarEDF::nodesToExplore(int node){
        std::cout << "node: " << node << std::endl;
    }

PathData AStarEDF::findPath(const Vec3i &_source, const Vec3i &_target)
{
    Node *current = nullptr;

    bool solved{false};

    discrete_world_.getNodePtr(_source)->parent = new Node(_source);
    discrete_world_.setOpenValue(_source, true);
    //Timer to record the execution time, not 
    //really important
    utils::Clock main_timer;
    main_timer.tic();

    line_of_sight_checks_ = 0;
    
    node_by_cost&     indexByCost          = openSet_.get<IndexByCost>();
    node_by_position& indexByWorldPosition = openSet_.get<IndexByWorldPosition>();

    indexByCost.insert(discrete_world_.getNodePtr(_source));
    
    while (!indexByCost.empty()) {
        //Get the element at the start of the open set ordered by cost
        auto it = indexByCost.begin();
        current = *it;
        indexByCost.erase(indexByCost.begin());
        
        if (current->coordinates == _target) { solved = true; break; }
        
        closedSet_.push_back(current);
        //This flags are used to avoid search in the containers, 
        //for speed reasons.
        current->isInOpenList = false;
        current->isInClosedList = true;

#if defined(ROS) && defined(PUB_EXPLORED_NODES)
        publishROSDebugData(current, indexByCost, closedSet_);
#endif

        // exploreNeighbours(current, _target, indexByWorldPosition);     
        // EXPLORING NEIGHBOURS CONSIDERING THE EDF GRADIENT
        exploreNeighbours_Gradient(current, _target, indexByWorldPosition);
    }
    main_timer.toc();
    
    PathData result_data = createResultDataObject(current, main_timer, closedSet_.size(), 
                                                 solved, _source, line_of_sight_checks_);
   //Clear internal variables. This should be done
   //the same way in every new algorithm implemented.
#if defined(ROS) && defined(PUB_EXPLORED_NODES)
    explored_node_marker_.points.clear();
#endif
    closedSet_.clear();
    openSet_.clear();
    delete discrete_world_.getNodePtr(_source)->parent;
    
    discrete_world_.resetWorld();
    return result_data;
}
}

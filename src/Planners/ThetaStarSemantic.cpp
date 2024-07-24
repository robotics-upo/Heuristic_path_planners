#include "Planners/ThetaStarSemantic.hpp"

namespace Planners
{
    ThetaStarSemantic::ThetaStarSemantic(bool _use_3d):ThetaStar(_use_3d, "thetastar_semantic") {}
    ThetaStarSemantic::ThetaStarSemantic(bool _use_3d, std::string _name = "thetastar_semantic" ):ThetaStar(_use_3d, _name) {}

    inline void ThetaStarSemantic::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        line_of_sight_checks_++;
        if (LineOfSight::bresenham3D(_s_aux->parent, _s2_aux, discrete_world_, checked_nodes))  
        {
            auto dist2   = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
            auto edge2   = ComputeSemanticEdgeCost(checked_nodes, _s_aux->parent, _s2_aux);
            if (edge2 == 0)
                edge2 = 1;

            line_of_sight_checks_++;
            LineOfSight::bresenham3D(_s_aux, _s2_aux, discrete_world_, checked_nodes_current);

            auto dist1   = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);  
            auto edge1   =  ComputeSemanticEdgeCost(checked_nodes_current, _s_aux, _s2_aux);
            if (edge1 == 0)
                edge1 = 1;

            if ( ( _s_aux->parent->G + (dist2 * edge2) ) < ( _s_aux->G + (dist1 * edge1))) 
            {
                _s2_aux->parent = _s_aux->parent;
                _s2_aux->G      = _s_aux->parent->G + (dist2 * edge2);  // This is the same than A*
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
                _s2_aux->C      = edge2 * dist2;
            }
            else{
                _s2_aux->parent =_s_aux;
                _s2_aux->G      = _s_aux->G + (dist1 * edge1);   // This is the same than A*      
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
                _s2_aux->C      = edge1 * dist1; 
            }
        } else {

            _s2_aux->parent=_s_aux;

            line_of_sight_checks_++;
            LineOfSight::bresenham3D(_s_aux, _s2_aux, discrete_world_, checked_nodes);
            
            auto dist1 = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);  
            auto edge1 =  ComputeSemanticEdgeCost(checked_nodes, _s_aux, _s2_aux);
            if (edge1 == 0)
                edge1 = 1;

            _s2_aux->G     =  _s_aux->G + (dist1 * edge1);  // This is the same than A*
            _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
            _s2_aux->C     =  edge1 * dist1;
        }
        checked_nodes->clear();
        checked_nodes_current->clear();
    }

    inline unsigned int ThetaStarSemantic::ComputeSemanticEdgeCost(const utils::CoordinateListPtr _checked_nodes, const Node* _s, const Node* _s2){ 
        
        double cost_semantic{0};
        double mean_cost_semantic{0};

        float c_wall, c_door, c_colum, c_furnish, c_stair, c_panel, c_lamp, c_glass;
        // float coef, c_wall, c_door, c_colum, c_furnish, c_stair, c_panel, c_lamp, c_glass;
        // coef=1;
        c_wall=coef;
        c_door=3*coef;
        c_colum=2.5*coef;
        c_furnish=1*coef;
        // c_stair=1;
        c_stair=1.5*coef;
        c_panel=1.5*coef;
        c_lamp=2*coef;
        c_glass=2*coef;
            
        auto n_checked_nodes = _checked_nodes->size();
        
        if( n_checked_nodes >= 1 ){
            // std::cout << "Nodes: " << n_checked_nodes << std::endl;
            for(auto &it: *_checked_nodes){
                // if (discrete_world_.getNodePtr(it)->semantic !=0){
                //     std::cout << "COSTE: " << discrete_world_.getNodePtr(it)->cost << std::endl;
                //     std::cout << "COSTE SEMANTICO: " << discrete_world_.getNodePtr(it)->semantic << std::endl;
                //     usleep(1e4);
	            //     std::cout << "Please a key to go to the next iteration..." << std::endl;
	            //     getchar(); // Comentar para no usar tecla.
                // }

                if (discrete_world_.getNodePtr(it)->semantic == 1)
                    cost_semantic += c_wall;
                else if (discrete_world_.getNodePtr(it)->semantic == 2)
                    cost_semantic += c_door;
                else if (discrete_world_.getNodePtr(it)->semantic == 3)
                    cost_semantic += c_colum;
                else if (discrete_world_.getNodePtr(it)->semantic == 4)
                    cost_semantic += c_furnish;
                else if (discrete_world_.getNodePtr(it)->semantic == 5)
                    cost_semantic += c_stair;
                else if (discrete_world_.getNodePtr(it)->semantic == 6)
                    cost_semantic += c_panel;
                else if (discrete_world_.getNodePtr(it)->semantic == 7)
                    cost_semantic += c_lamp;
                else if (discrete_world_.getNodePtr(it)->semantic == 8)
                    cost_semantic += c_glass;

            }
        }
        // if (cost_semantic != 0){
        //     std::cout << "cost_semantic: " << cost_semantic << std::endl;
        //     usleep(1e4);
	    //     std::cout << "Please a key to go to the next iteration..." << std::endl;
	    //     getchar(); // Comentar para no usar tecla.
        // }

        if( n_checked_nodes >= 1){
            if (_s->semantic == 1)
                cost_semantic += c_wall;
            else if (_s->semantic == 2)
                cost_semantic += c_door;
            else if (_s->semantic == 3)
                cost_semantic += c_colum;
            else if (_s->semantic == 4)
                cost_semantic += c_furnish;
            else if (_s->semantic == 5)
                cost_semantic += c_stair;
            else if (_s->semantic == 6)
                cost_semantic += c_panel;
            else if (_s->semantic == 7)
                cost_semantic += c_lamp;
            else if (_s->semantic == 8)
                cost_semantic += c_glass;
            
            if (_s2->semantic == 1)
                cost_semantic += c_wall;
            else if (_s2->semantic == 2)
                cost_semantic += c_door;
            else if (_s2->semantic == 3)
                cost_semantic += c_colum;
            else if (_s2->semantic == 4)
                cost_semantic += c_furnish;
            else if (_s2->semantic == 5)
                cost_semantic += c_stair;
            else if (_s2->semantic == 6)
                cost_semantic += c_panel;
            else if (_s2->semantic == 7)
                cost_semantic += c_lamp;
            else if (_s2->semantic == 8)
                cost_semantic += c_glass;
            mean_cost_semantic = cost_semantic/(n_checked_nodes+1); // Dividir entre el numero de nodos
            // mean_cost_semantic = cost_semantic/(n_checked_nodes+1);         }
        }
        // checked_nodes == 0
        else{
            cost_semantic = 0;
            if (_s->semantic == 1)
                cost_semantic += c_wall;
            else if (_s->semantic == 2)
                cost_semantic += c_door;
            else if (_s->semantic == 3)
                cost_semantic += c_colum;
            else if (_s->semantic == 4)
                cost_semantic += c_furnish;
            else if (_s->semantic == 5)
                cost_semantic += c_stair;
            else if (_s->semantic == 6)
                cost_semantic += c_panel;
            else if (_s->semantic == 7)
                cost_semantic += c_lamp;
            else if (_s->semantic == 8)
                cost_semantic += c_glass;
            else
                cost_semantic=0;
            
            if (_s2->semantic == 1)
                cost_semantic += c_wall;
            else if (_s2->semantic == 2)
                cost_semantic += c_door;
            else if (_s2->semantic == 3)
                cost_semantic += c_colum;
            else if (_s2->semantic == 4)
                cost_semantic += c_furnish;
            else if (_s2->semantic == 5)
                cost_semantic += c_stair;
            else if (_s2->semantic == 6)
                cost_semantic += c_panel;
            else if (_s2->semantic == 7)
                cost_semantic += c_lamp;
            else if (_s2->semantic == 8)
                cost_semantic += c_glass;
            else
                cost_semantic=0;
            mean_cost_semantic = cost_semantic/2;  // Dividir entre el numero de nodos
            // mean_cost_semantic = cost_semantic;
        }
        // return static_cast<unsigned int>( mean_cost_semantic * dist_scale_factor_reduced_);
        // std::cout << "edge: " << mean_cost_semantic << std::endl;
        return static_cast<unsigned int>( mean_cost_semantic);
    }
    
    inline unsigned int ThetaStarSemantic::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs){
        
        unsigned int cost = _current->G;
        unsigned int cost2 = 0;

        float c_wall, c_door, c_colum, c_furnish, c_stair, c_panel, c_lamp, c_glass;
        // float coef, c_wall, c_door, c_colum, c_furnish, c_stair, c_panel, c_lamp, c_glass;
        // coef=1;
        c_wall=coef;
        c_door=3*coef;
        c_colum=2.5*coef;
        c_furnish=1*coef;
        // c_stair=1;
        c_stair=1.5*coef;
        c_panel=1.5*coef;
        c_lamp=2*coef;
        c_glass=2*coef;

        if(_dirs  == 8){
            cost2 += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
        }else{
            cost2 += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
        }

        // EDF COST --> ThetarStarM1
        // auto cost_term = static_cast<unsigned int>(cost_weight_ * _suc->cost * dist_scale_factor_reduced_);
        // cost   += cost_term;
        // _suc->C = cost_term;

        // SEMANTIC COST
        // Wall
        if (_suc->semantic == 1){
            cost2=c_wall*cost2;
            // std::cout << "cost: " << _suc->cost << std::endl;
            // std::cout << "semantic: " << _suc->semantic << std::endl;
            // usleep(1e4);
	        // std::cout << "Please a key to go to the next iteration..." << std::endl;
	        // getchar(); // Comentar para no usar tecla.
        }
        // Door
        else if (_suc->semantic == 2){
            cost2=c_door*cost2;
        }

        // Colum
        else if (_suc->semantic == 3){
            cost2=c_colum*cost2;
        }
        // Furnishing
        else if (_suc->semantic == 4){
            cost2=c_furnish*cost2;
        }
        // Stairs
        else if (_suc->semantic == 5){
            cost2=c_stair*cost2;
        }
        // Panels --> Barandilla
        else if (_suc->semantic == 6){
            cost2=c_panel*cost2;
        }
        // Lamp
        else if (_suc->semantic == 7){
            cost2=c_lamp*cost2;
        }
        // Glass wall
        else if (_suc->semantic == 8){
            cost2=c_glass*cost2;
        }

        cost = cost + cost2;

        _suc->C = _suc->cost;

        return cost;
    }
}

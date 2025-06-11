#ifndef TPE_COMMON_H
#define TPE_COMMON_H

#include <vector>

//struct point2D_t {

//    double cx,cy; /*coordonnees du centre de gravite de la tache */

//};

/*struct pose3D_t {

    double tx;
    double ty;
    double tz;

    double rx;
    double ry;
    double rz;


};

struct cible_t {
    int b_isEnabled ;
    std::vector<struct point2D_t> marker;
    std::vector<struct pose3D_t> pose;

};*/

struct camera_t {    
    double alpha_u;
    double alpha_v;
    double u0;
    double v0;
};

struct control_param_t {    
    double gain_t;
    double gain_r;
    double z_des;    
};

struct rotation_point_param_t {

    std::vector<double> position;    
};

struct target_param_t {
    
    std::vector<double> geometry;  
    std::vector<std::vector<double>> array_geometry;  
};



struct tpe_control_vision_t		{

    //struct cible_t cible[2]; /* information sur la cible de reference : 0 et le cible courante : 1*/
    double control[4];/**/

    struct camera_t camera_param;
    struct control_param_t control_param;
    struct rotation_point_param_t rotation_point_param;
    struct target_param_t target_param;
 
    };

#endif

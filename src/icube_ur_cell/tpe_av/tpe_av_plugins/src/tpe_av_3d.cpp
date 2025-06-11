#include "tpe_av_plugins/tpe_av_plugins.hpp"
#include "tpe_robot_mathlib/mathlib.hpp"
#include <iostream>
#include <math.h> 


using namespace tpe_robot_mathlib;

namespace tpe_av_plugins
{

TpeAv3D::TpeAv3D()
{
}

TpeAv3D::~TpeAv3D()
{
}

void TpeAv3D::initialize()
{
}
int TpeAv3D::Update_aoi ( struct tpe_t *tpe, int obj ) 
{
    for (unsigned int i =0; i < tpe->cibleN[obj].nb_marque; i++) 
        update_aoi_n(tpe->cibleN[obj].marque[i], tpe->im);
    
    return 0;    
}
int TpeAv3D::ImageProcessing ( struct tpe_t *tpe, int obj )
{
    for (unsigned int i =0; i < tpe->cibleN[obj].nb_marque; i++) 
        bary_n(tpe->cibleN[obj].marque[i], tpe->im);
    return 0;
}

int TpeAv3D::Update_mesure(struct tpe_t *tpe, struct tpe_control_vision_t tpe_control_, int obj_cur, int obj_des) 
{
    
    double u1,u2,v1,v2;
	double x01,x02,y01,y02;
	
	double **A, **Ainv;
	double *b;
	double *pose;
	
	double tz,tx,ty, alpha;

    double ALPHA_U,ALPHA_V,U_0,V_0;
    ALPHA_U =tpe_control_.camera_param.alpha_u;
    ALPHA_V =tpe_control_.camera_param.alpha_v;
    U_0 =tpe_control_.camera_param.u0;
    V_0 =tpe_control_.camera_param.v0;

	
	A = dmatrice (4,4);
	Ainv = dmatrice (4,4);
	b = dvect(4);
	pose = dvect(4);


	u1 = (tpe->cibleN[obj_cur].marque[0].cx-U_0)/ALPHA_U;
	u2 = (tpe->cibleN[obj_cur].marque[1].cx-U_0)/ALPHA_U;
	
	v1 = (tpe->cibleN[obj_cur].marque[0].cy-V_0)/ALPHA_V;
	v2 = (tpe->cibleN[obj_cur].marque[1].cy-V_0)/ALPHA_V;
	
	x01 = tpe_control_.target_param.geometry[0];
	x02 = tpe_control_.target_param.geometry[2];

	y01 = tpe_control_.target_param.geometry[1];
	y02 = tpe_control_.target_param.geometry[3];
	
	
	b[0] = u1; b[2] = u2;
	b[1] = v1; b[3] = v2;

	A[0][0]=1;A[0][1]=x01;A[0][2]=-y01;A[0][3]=0;
	A[1][0]=0;A[1][1]=y01;A[1][2]=x01;A[1][3]=1;
	A[2][0]=1;A[2][1]=x02;A[2][2]=-y02;A[2][3]=0;
	A[3][0]=0;A[3][1]=y02;A[3][2]=x02;A[3][3]=1;

	/*A[0][0]=1;A[0][1]=x01;A[0][2]=y01;A[0][3]=0;
	A[1][0]=0;A[1][1]=-y01;A[1][2]=x01;A[1][3]=1;
	A[2][0]=1;A[2][1]=x02;A[2][2]=y02;A[2][3]=0;
	A[3][0]=0;A[3][1]=-y02;A[3][2]=x02;A[3][3]=1;*/

	pinvGreville(A,4,4,Ainv);
	mxv(Ainv,b,pose,4,4);
	
	tz = 1.0/sqrt(pose[1]*pose[1] + pose[2]*pose[2]);
	tx = tz *pose[0];
	ty = tz*pose[3];

	alpha = atan2(pose[2],pose[1]);
	
	//printf(" pose :tx:%f ty:%f tz:%f alpha:%f\n",tx,ty,tz,alpha);
	


    tpe->info_image[obj_cur][0] = tx;
    tpe->info_image[obj_cur][1] = ty;
    tpe->info_image[obj_cur][2] = tz;
    tpe->info_image[obj_cur][3] = pose[2]*tz;
    tpe->info_image[obj_cur][4] = pose[1]*tz;

    /*
    measure->clear();
    measure->push_back(tx);
    measure->push_back(ty);
    measure->push_back(tz);
    measure->push_back(pose[2]*tz);
    measure->push_back(pose[1]*tz);*/
	
	Detruitdmatrice(A,4);
	Detruitdmatrice(Ainv,4);
	Detruitdvect(b);
	Detruitdvect(pose);

    return 0;
}
int TpeAv3D::Commande ( struct tpe_t tpe, struct tpe_control_vision_t tpe_control_, double *control) 
{
	
    double err[4],ContObj[4];

	int i;
	double **Tco,**Tcod, **Toc, **Tood, **Tdo, **Tdc, **Tcd, **Todc, **Todo;
    double measure[5];
    double reference[5];



	Tco = dmatrice (4,4);
	Tdo = dmatrice (4,4);
	Tdc = dmatrice (4,4);
	Tcd = dmatrice (4,4);
	Tcod = dmatrice (4,4);
    Todc = dmatrice (4,4);
	Toc = dmatrice (4,4);
	Tood = dmatrice (4,4);
    Todo = dmatrice (4,4);

    double GAIN_T,GAIN_R;
    double p_rot[3];


    if (!tpe.cibleN[0].b_isEnabled || !tpe.cibleN[1].b_isEnabled) {
        return 1;
    }

    /*for (int i =0; i < 3 ;i++) {
        p_rot[i] = tpe_control_.rotation_point_param.position[i];
    }*/

    for (int i =0 ; i < 5; i++) {

        measure[i] = tpe.info_image[1][i];
        reference[i] = tpe.info_image[0][i];
    }

	Tco[0][0] =measure[4];	Tco[0][1] = -measure[3];	Tco[0][2] =0;	Tco[0][3] =measure[0];
	Tco[1][0] =measure[3];	Tco[1][1] =measure[4];	    Tco[1][2] =0;	Tco[1][3] =measure[1];
	Tco[2][0] =0;			Tco[2][1] =0;				Tco[2][2] =1;	Tco[2][3] =measure[2];
	Tco[3][0] =0;			Tco[3][1] =0;               Tco[3][2] =0;	Tco[3][3] =1;

	Tcod[0][0] =reference[4];	Tcod[0][1] = -reference[3];	Tcod[0][2] =0;	Tcod[0][3] =reference[0]; 
	Tcod[1][0] =reference[3];	Tcod[1][1] =reference[4];	Tcod[1][2] =0;	Tcod[1][3] =reference[1]; 
	Tcod[2][0] =0;				Tcod[2][1] =0;				Tcod[2][2] =1;	Tcod[2][3] =reference[2]; 
	Tcod[3][0] =0;				Tcod[3][1] =0;              Tcod[3][2] =0;	Tcod[3][3] =1;

	/*Tdo[0][0] =reference[4];	Tdo[0][1] = -reference[3];	Tdo[0][2] =0;	Tdo[0][3] =reference[0]; 
	Tdo[1][0] =reference[3];	Tdo[1][1] =reference[4];	Tdo[1][2] =0;	Tdo[1][3] =reference[1]; 
	Tdo[2][0] =0;				Tdo[2][1] =0;				Tdo[2][2] =1;	Tdo[2][3] =reference[2]; 
	Tdo[3][0] =0;				Tdo[3][1] =0;               Tdo[3][2] =0;	Tdo[3][3] =1;*/


	//pinvGreville(Tcod,4,4,Todc);
    
    pinvGreville(Tco,4,4,Toc);
    //printf("measure : %f %f %f\n", Toc[0][3], Toc[1][3], Toc[2][3]);

	mxm(Toc,Tcod,Tood,4,4,4);


	/*mxm(Tdo,Toc,Tdc,4,4,4);
	pinvGreville(Tdc,4,4,Tcd);*/
 
	
	//erreur en position
	for (i = 0; i < 3; i++ )  
		err[i] = Tood[i][3];
		//err[i] = Tdc[i][3];
		//err[i] = Tcd[i][3];
    //err[0] = Tcd[0][0]*reference[0] + Tcd[0][1]*reference[1] - measure[0];
    //err[1] = Tcd[1][0]*reference[0] + Tcd[1][1]*reference[1] - measure[1];
    //err[2] = reference[2]  - measure[2];
	
	//erreur en rotation
	err[3] = atan2( Tood[1][0],Tood[0][0] );
	//err[3] = atan2( Tdc[1][0],Tdc[0][0] );
	//err[3] = atan2( Tcd[1][0],Tcd[0][0] );

    printf("reference : %f %f %f\n", reference[0], reference[1], reference[2]);
    printf("measure : %f %f %f\n", measure[0], measure[1], measure[2]);

    printf("erreur : %f %f %f %f \n", err[0], err[1], err[2], err[3]);

    GAIN_T = tpe_control_.control_param.gain_t;
    GAIN_R = tpe_control_.control_param.gain_r;

	//commande en rotation
	ContObj[3] = GAIN_R*err[3];
	
	//commande en position
	for (i=0; i < 3; i++)
		ContObj[i] = GAIN_T * err[i];
	
	//ContObj[0] +=p_rot[1]*ContObj[3]; 

	/*tpe_control_.control[0] = Tco[0][0]*ContObj[0] + Tco[0][1]*ContObj[1];
	tpe_control_.control[1] = Tco[1][0]*ContObj[0] + Tco[1][1]*ContObj[1];
	tpe_control_.control[2] = ContObj[2];
	tpe_control_.control[3] = ContObj[3];*/

	//control[0] = Tco[0][0]*ContObj[0] + Tco[0][1]*ContObj[1];
	//control[1] = Tco[1][0]*ContObj[0] + Tco[1][1]*ContObj[1];
	//control[2] = ContObj[2];
	//control[3] = ContObj[3];
	//control[0] = Tdc[0][0]*ContObj[0] + Tdc[1][0]*ContObj[1];
	//control[1] = Tdc[0][1]*ContObj[0] + Tdc[1][1]*ContObj[1];
	control[0] = ContObj[0];
	control[1] = ContObj[1];
	control[2] = ContObj[2];
	control[3] = ContObj[3];

    printf("control : %f %f %f %f \n", control[0], control[1], control[2], control[3]);

	Detruitdmatrice(Tco,4);
	Detruitdmatrice(Tdo,4);
	Detruitdmatrice(Tdc,4);
	Detruitdmatrice(Tcd,4);
	Detruitdmatrice(Tcod,4);
	Detruitdmatrice(Todc,4);
	Detruitdmatrice(Toc,4);
	Detruitdmatrice(Tood,4);
    Detruitdmatrice(Todo,4);

    
    return 0;
}

int TpeAv3D::bary_n (  struct marque_t &marque_ , struct image_t im_) {
    
    marque_.m00 =0.0;
    marque_.m10 =0.0;marque_.m01 =0.0;
    marque_.m20 =0.0;marque_.m02 =0.0;marque_.m11 =0.0;
    marque_.mu20 =0.0;marque_.mu02 =0.0;marque_.mu11 =0.0;

    
    for (int j = marque_.aoi_y0; 
         j < marque_.aoi_y0 + marque_.aoi_dy; 
         j++)	{

        for (int i = marque_.aoi_x0; 
                 i < marque_.aoi_x0+marque_.aoi_dx;
                 i++)	{

            //if ( im_.buf[j*im_.width + i ] < TPE_IMAGE_PROCESSING_THRESHOLD )	{
            if ( im_.coord[j][i] < TPE_IMAGE_PROCESSING_THRESHOLD )	{

                marque_.m00 += 1.0;
                marque_.m10 += (double)i;
                marque_.m01 += (double)j;
                marque_.m20 += (double)i*i;
                marque_.m02 += (double)j*j;
                marque_.m11 += (double)i*j;


            }
        }
    }

    if ( marque_.m00 )	{

        marque_.cx = marque_.m10/marque_.m00;
        marque_.cy = marque_.m01/marque_.m00;
        marque_.mu20 = marque_.m20 - marque_.m10*marque_.m10/marque_.m00;
        marque_.mu02 = marque_.m02 - marque_.m01*marque_.m01/marque_.m00;
        marque_.mu11 = marque_.m11 - marque_.m10*marque_.m01/marque_.m00;

        
    }
    else	return 1;

    return 0 ;
}
int TpeAv3D::update_aoi_n ( struct marque_t &marque_, struct image_t im_ ) {
    
    double r;

    //les marques sont des cercles
    // calcul du rayon s = pi*r2
    if ( marque_.m00 > TPE_IMAGE_PROCESSING_MINIMAL_SURFACE )
        r = sqrt(marque_.m00/M_PI);


    marque_.aoi_dx = 2*r + 20;
    marque_.aoi_dy = 2*r + 20;


    if (marque_.aoi_dx > im_.width)
        marque_.aoi_dx = im_.width;

    if (marque_.aoi_dy > im_.height)
        marque_.aoi_dy = im_.height;


    if ( marque_.cx - marque_.aoi_dx / 2  < 0 )
        marque_.aoi_x0 = 0;

    else if ( marque_.cx + marque_.aoi_dx / 2 >= im_.width  ) {
        marque_.aoi_x0 = im_.width - marque_.aoi_dx ;
        if (marque_.aoi_x0 < 0) marque_.aoi_x0 = 0;
    }
    else
        marque_.aoi_x0 = marque_.cx - marque_.aoi_dx / 2;



    if ( marque_.cy - (marque_.aoi_dy / 2) < 0  )
        marque_.aoi_y0 = 0;
    else if ( marque_.cy  + marque_.aoi_dy / 2 >= (im_.height ) ) {
        marque_.aoi_y0 = im_.height - marque_.aoi_dy ;
        if (marque_.aoi_y0 < 0) marque_.aoi_y0 = 0;
    }
    else
        marque_.aoi_y0 = marque_.cy - marque_.aoi_dy / 2;


    return 0;

}



}  // namespace tpe_av_plugins
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tpe_av_plugins::TpeAv3D, tpe_base::TpeBase)


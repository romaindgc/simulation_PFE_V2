#ifndef TPE_H
#define TPE_H

#define TPE_NB_OBJET		2  /* Nb d'objet ï¿½detecter (cible desiree : 0 cible courante : 1 */
#define TPE_NB_MARQUEUR		2  /* Nombre de marqueurs dans l'objet */
#define TPE_NB_JOINTS       4  /* Nombre de dof du robot */
#define TPE_SEUIL		100
#define TPE_MARGE		20

#define TPE_IMAGE_PROCESSING_THRESHOLD 90
#define TPE_IMAGE_PROCESSING_MINIMAL_SURFACE 10


/* Estimation de la profondeur de la cible dans sa position de reference */
//#define Z_EST			0.9

/* Gain de la commande */
//#define GAIN_T			5.0
//#define GAIN_R			5.0

/* Paramï¿½res intrinseque de la camera */
//#define ALPHA_U			1000.0
//#define ALPHA_V			1000.0

//#define U_0			380.0
//#define V_0			285.0

#define TPE_PAS			0.08 /*pas de la cible*/


/* structure contenant la description de la tache */
struct cible_t	{

    int aoi_x0; /*origine de l'imagette haut gauche */
    int aoi_y0; /*origine de l'imagette haut gauche */

    int aoi_dx; /* largeur de l'imagette */
    int aoi_dy; /* hauteur de l'imagette haut gauche */

    int nb_pix; /*Nb de pixels detectes */

    double cx,cy; /*coordonnees du centre de gravite de la tache */
    double pt1x, pt1y; /**coordonnees d'un point particulier de la tache */
    int nb_marque; /* nombre de marqueurs initialisï¿½s de l'objet */

    double m00;
    double m10,m01;
    double m20,m11,m02;
    double mu20,mu11,mu02;

    int b_isEnabled;
    int b_isTracking;

    };

struct marque_t {

    int aoi_x0; /*origine de l'imagette haut gauche */
    int aoi_y0; /*origine de l'imagette haut gauche */

    int aoi_dx; /* largeur de l'imagette */
    int aoi_dy; /* hauteur de l'imagette haut gauche */

    int nb_pix; /*Nb de pixels detectes */

    double cx,cy; /*coordonnees du centre de gravite de la tache */
    double pt1x, pt1y; /**coordonnees d'un point particulier de la tache */
    int nb_marque; /* nombre de marqueurs initialisï¿½s de l'objet */

    double m00;
    double m10,m01;
    double m20,m11,m02;
    double mu20,mu11,mu02;

};

struct cible_base_t {
    int nb_marque; /* nombre de marqueurs initialisï¿½s de l'objet */
    int b_isEnabled;
    int b_isTracking;
    struct marque_t marque[TPE_NB_MARQUEUR];
};

/* structure contenant les infos de l'image courante ï¿½traiter */

struct image_t	{

    unsigned short width; /* largeur de l'image */
    unsigned short height; /* hauteur de l'image */
    unsigned char *buf; /* pointeur sur l'image */
    unsigned char **coord; /*pointeur sur les coordonnï¿½s de l'image */
    unsigned char *buf_save; /* pointeur sur l'image */
    unsigned char **coord_save; /*pointeur sur les coordonnï¿½s de l'image */

    };


struct tpe_t		{

    struct image_t im; /* image courante */
    struct cible_t cible[TPE_NB_OBJET]; /* information sur la cible de reference : 0 et le cible courante : 1*/
    struct cible_base_t cibleN[TPE_NB_OBJET];
    double info_image[TPE_NB_OBJET][6]; /*informations visuelles reconstruite ï¿½partir des moments de l'objet */
    int nb_objet; /* nombre d'objet initialisï¿½*/

    };

struct control_t {

    double gain_t;
    double gain_r;
    double z_des;

};

#endif // TPE_H


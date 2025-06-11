#ifndef TPE_AV_PLUGINS__TPE_AV_PLUGINS_HPP_
#define TPE_AV_PLUGINS__TPE_AV_PLUGINS_HPP_

//#include "tpe_av_plugins/visibility_control.h"
#include "tpe_base/tpe_base.h"

namespace tpe_av_plugins
{

class TpeAv3D : public tpe_base::TpeBase
{
public:
  TpeAv3D();
  
  void initialize();
  int Update_aoi ( struct tpe_t *tpe, int obj );
  int ImageProcessing ( struct tpe_t *tpe, int obj ) ;
  int Update_mesure(struct tpe_t *tpe, struct tpe_control_vision_t tpe_control_, int obj_cur, int obj_des=0) ;
  int Commande ( struct tpe_t tpe, struct tpe_control_vision_t tpe_control_, double *control) ;


  virtual ~TpeAv3D();

private :
  int bary_n (struct marque_t &marque_, struct image_t im_ );
  int update_aoi_n ( struct marque_t &marque_, struct image_t im_);

};

}  // namespace tpe_av_plugins

#endif  // TPE_AV_PLUGINS__TPE_AV_PLUGINS_HPP_

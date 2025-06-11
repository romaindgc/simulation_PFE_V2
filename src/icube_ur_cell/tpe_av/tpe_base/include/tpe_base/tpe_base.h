#ifndef TPE_BASE_H
#define TPE_BASE_H

#include "tpe.h"
#include "tpe_common.hpp"

namespace tpe_base {
class TpeBase
{
public:
    
    virtual void initialize() = 0;
    virtual int Update_aoi ( struct tpe_t *tpe, int obj ) = 0;
    virtual int ImageProcessing ( struct tpe_t *tpe, int obj ) = 0;
    virtual int Update_mesure(struct tpe_t *tpe, struct tpe_control_vision_t tpe_control_, int obj_cur, int obj_des=0) = 0;
    virtual int Commande ( struct tpe_t tpe, struct tpe_control_vision_t tpe_control_, double *control) = 0;

    //double z_des;

protected:
    TpeBase() {}

};
} //namespace tpe_base

#endif // TPE_BASE_H

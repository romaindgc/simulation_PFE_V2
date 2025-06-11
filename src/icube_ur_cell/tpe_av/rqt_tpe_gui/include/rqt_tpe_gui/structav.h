#ifndef STRUCTAV_H
#define STRUCTAV_H

#define MARQUE_COUNT_MAX    4
#include <QList>
#include <QPointF>
#include <QRect>




typedef struct {

    bool b_isEnabled;
    QRect aoi;
    QPointF centroid;

    double threshold;


} STRUCT_POINT_AV;

typedef struct  {

    bool b_isEnabled;
    int PointCount;
    QList <STRUCT_POINT_AV> listPoint;


} STRUCT_TARGET_AV;


#endif // STRUCTAV_H

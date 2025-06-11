
#include "rqt_tpe_gui/GraphicsScene.h"

#include <QDebug>

//#include <QGraphicsEllipseItem>
//#include <QGraphicsPathItem>
//#include <QPainterPath>

GraphicsScene::GraphicsScene(QObject *parent) :
    QGraphicsScene(parent)
{
    this->setBackgroundBrush(Qt::gray);
    leftButtonPress=false;
    rightButtonPress=false;

}

void GraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent * mouseEvent)
{

    //qDebug() << Q_FUNC_INFO << mouseEvent->buttons();
    switch (mouseEvent->buttons())    {
    case Qt::LeftButton :
        //qDebug() << Q_FUNC_INFO << "Left button clicked";
        leftButtonPress=true;
        rightButtonPress=false;

        emit clickedPoint( (mouseEvent->scenePos().toPoint()) );

        break;
    case Qt::RightButton :
        //qDebug() << Q_FUNC_INFO << "Right button clicked";
        leftButtonPress=false;
        rightButtonPress=true;

        break;



    }


    QGraphicsScene::mousePressEvent(mouseEvent);
}

void GraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent * mouseEvent)   {

    emit setMouseCursorPos((mouseEvent->scenePos().toPoint()));
    QGraphicsScene::mouseMoveEvent(mouseEvent);


}

void GraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent * mouseEvent)    {


    qDebug() << Q_FUNC_INFO << mouseEvent->buttons();
    if (leftButtonPress && !rightButtonPress)   {
        qDebug() << Q_FUNC_INFO << "Left button release";
    }
    else if (!leftButtonPress && rightButtonPress)  {
        qDebug() << Q_FUNC_INFO << "right button release";
        emit PointSelectionValidation();
    }
    else
        qDebug() << Q_FUNC_INFO << "unknown state";

    leftButtonPress=false;
    rightButtonPress=false;



    QGraphicsScene::mouseReleaseEvent(mouseEvent);
}


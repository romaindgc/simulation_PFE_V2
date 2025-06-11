#ifndef GRAPHICSSCENE_H
#define GRAPHICSSCENE_H

#include <QObject>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QPointF>

class GraphicsScene : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit GraphicsScene(QObject *parent = 0);

    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent * mouseEvent);
    virtual void mousePressEvent(QGraphicsSceneMouseEvent * mouseEvent);

protected :
    virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent * mouseEvent);

signals:

    void clickedPoint(QPoint p);
    void setMouseCursorPos(QPoint p);
    void PointSelectionValidation();


public slots:

private :
    bool leftButtonPress;
    bool rightButtonPress;

};

#endif // GRAPHICSSCENE_H

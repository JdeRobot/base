#ifndef CONTROLVW_H
#define CONTROLVW_H

#include <QtGui>

#include <iostream>
#include <string>

#include "easyiceconfig/EasyIce.h"

class controlVW : public QWidget
{
    Q_OBJECT
public:
    explicit controlVW(QWidget *parent = 0);
    
    void Stop();
    void setIC(Ice::CommunicatorPtr ic);
	float getV();
	float getW();

private:
    QPointF line;
    QImage qimage;

	float v, w, v_max, w_max;
	Ice::CommunicatorPtr ic;

protected:
    void paintEvent(QPaintEvent *);
    void mouseMoveEvent ( QMouseEvent * event );

signals:
    void VW_changed(float, float);

public slots:
    
};

#endif // CONTROLVW_H

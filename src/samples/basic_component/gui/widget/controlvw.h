#ifndef CONTROLVW_H
#define CONTROLVW_H

#include <QtWidgets>
#include <iostream>
#include <string>
#include <jderobot/comm/communicator.hpp>

class controlVW : public QWidget
{
    Q_OBJECT
public:
    explicit controlVW(QWidget *parent = 0);
    
    void Stop();
    void setjdrc(Comm::Communicator* jdrc);
	float getV();
	float getW();

private:
    QPointF line;
    QImage qimage;

	float v, w, v_max, w_max;
	Comm::Communicator* jdrc;

protected:
    void paintEvent(QPaintEvent *);
    void mouseMoveEvent ( QMouseEvent * event );

signals:
    void VW_changed(float, float);
    
};

#endif // CONTROLVW_H

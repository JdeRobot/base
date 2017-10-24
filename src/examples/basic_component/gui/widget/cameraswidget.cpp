#include "cameraswidget.h"

CamerasWidget::CamerasWidget(Comm::CameraClient* camera)
{

    this->camera = camera;

    QGridLayout* mainLayout = new QGridLayout();

    labelImage = new QLabel();

    mainLayout->addWidget(labelImage, 0, 0);

    setLayout(mainLayout);

    setWindowTitle("Camera");

}

void CamerasWidget::update()
{
    

    JdeRobotTypes::Image rgb;
    rgb = this->camera->getImage();

    cv::Mat frame1 = rgb.data;
    if (!frame1.empty()){

        QImage imageQt1 = QImage((const unsigned char*)(frame1.data),
                            frame1.cols,
                            frame1.rows,
                            frame1.step,
                            QImage::Format_RGB888);

        labelImage->setPixmap(QPixmap::fromImage(imageQt1));
    }
}

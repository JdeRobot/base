#include "cameraswidget.h"

CamerasWidget::CamerasWidget(jderobot::cameraClient* camera)
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
    cv::Mat frame1;
    this->camera->getImage(frame1);

    //cv::resize(frame1, frame1, cv::Size(320, 240));

    QImage imageQt1 = QImage((const unsigned char*)(frame1.data),
                            frame1.cols,
                            frame1.rows,
                            frame1.step,
                            QImage::Format_RGB888);

    labelImage->setPixmap(QPixmap::fromImage(imageQt1));
}

#ifndef JDEROBOTTYPES_H
#define JDEROBOTTYPES_H

class LaserD
{
public:
    std::vector<float> values;
    double minAngle = 0;
    double maxAngle = 0;
    double minRange = 0;
    double maxRange = 0;
};

#endif // JDEROBOTTYPES_H
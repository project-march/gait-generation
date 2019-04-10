#include <march_gait_generator/widgets/FancySlider.h>

#include <QStyleOptionSlider>
#include <QToolTip>

FancySlider::FancySlider(QWidget * parent)
        : QSlider(parent)
{
}

FancySlider::FancySlider(Qt::Orientation orientation, QWidget * parent)
        : QSlider(orientation, parent)
{
}

void FancySlider::sliderChange(QAbstractSlider::SliderChange change)
{
    QSlider::sliderChange(change);

    if (change == QAbstractSlider::SliderValueChange )
    {
        QStyleOptionSlider opt;
        initStyleOption(&opt);

        QRect sr = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);
        QPoint bottomRightCorner = sr.bottomLeft();

        QToolTip::showText(mapToGlobal( QPoint( bottomRightCorner.x(), bottomRightCorner.y() ) ), QString::number((float)value()/MULTIPLICATION_FACTOR), this);
    }
}
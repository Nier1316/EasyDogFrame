#include <QApplication>
#include "motor_control_gui.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    MotorControlGUI window;
    window.show();

    return app.exec();
}

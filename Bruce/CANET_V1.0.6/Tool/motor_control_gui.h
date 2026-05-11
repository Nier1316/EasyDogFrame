/**
 * @file    motor_control_gui.h
 * @brief   电机控制上位机GUI - 用于计算和显示CAN指令
 */
#ifndef MOTOR_CONTROL_GUI_H
#define MOTOR_CONTROL_GUI_H

#include <QMainWindow>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QTextEdit>
#include <QLabel>
#include <QGroupBox>
#include <QRadioButton>
#include <QButtonGroup>

class MotorControlGUI : public QMainWindow {
    Q_OBJECT

public:
    MotorControlGUI(QWidget *parent = nullptr);
    ~MotorControlGUI();

private slots:
    // 电机选择
    void onCanPortChanged(int index);
    void onMotorIdChanged(int value);

    // 电机控制
    void onEnableMotor();
    void onDisableMotor();
    void onSetZero();
    void onClearError();

    // 模式选择
    void onControlModeChanged(int id);
    void onSetControlMode();  // 新增：设置控制模式

    // 参数计算
    void onCalculateImpedanceCommand();
    void onCalculateSpeedCommand();
    void onCalculatePositionCommand();

    // 参数读写
    void onReadParameter();
    void onWriteParameter();

private:
    // 初始化UI
    void initUI();
    void createMotorSelectionGroup();
    void createMotorControlGroup();
    void createControlModeGroup();
    void createParameterGroup();
    void createParameterRWGroup();
    void createResultGroup();

    // 辅助函数
    void displayCanFrame(const uint8_t* data, int length, const QString& title);
    void clearResults();
    void updateParameterUI();

    // UI组件 - 电机选择
    QComboBox *canPortCombo;
    QSpinBox *motorIdSpinBox;
    QLabel *selectedMotorLabel;

    // UI组件 - 电机控制
    QPushButton *enableBtn;
    QPushButton *disableBtn;
    QPushButton *setZeroBtn;
    QPushButton *clearErrorBtn;

    // UI组件 - 控制模式
    QRadioButton *impedanceRadio;
    QRadioButton *speedRadio;
    QRadioButton *positionRadio;
    QButtonGroup *modeGroup;
    QPushButton *setModeBtn;  // 新增：设置模式按钮

    // UI组件 - 阻抗模式参数
    QGroupBox *impedanceGroup;
    QDoubleSpinBox *impedancePosSpinBox;
    QDoubleSpinBox *impedanceVelSpinBox;
    QDoubleSpinBox *impedanceKpSpinBox;
    QDoubleSpinBox *impedanceKdSpinBox;
    QDoubleSpinBox *impedanceTorqueSpinBox;
    QPushButton *impedanceCalcBtn;

    // UI组件 - 速度模式参数
    QGroupBox *speedGroup;
    QDoubleSpinBox *speedVelSpinBox;
    QDoubleSpinBox *speedKpSpinBox;
    QDoubleSpinBox *speedKiSpinBox;
    QPushButton *speedCalcBtn;

    // UI组件 - 位置模式参数
    QGroupBox *positionGroup;
    QDoubleSpinBox *positionPosSpinBox;
    QDoubleSpinBox *positionKvpSpinBox;
    QDoubleSpinBox *positionKpSpinBox;
    QDoubleSpinBox *positionKdSpinBox;
    QDoubleSpinBox *positionKviSpinBox;
    QPushButton *positionCalcBtn;

    // UI组件 - 参数读写
    QComboBox *paramTypeCombo;
    QDoubleSpinBox *paramValueSpinBox;
    QPushButton *readParamBtn;
    QPushButton *writeParamBtn;

    // UI组件 - 结果显示
    QTextEdit *resultTextEdit;
    QLabel *canFrameLabel;

    // 当前选择的电机
    uint8_t currentCanPort;
    uint8_t currentMotorId;
};

#endif // MOTOR_CONTROL_GUI_H

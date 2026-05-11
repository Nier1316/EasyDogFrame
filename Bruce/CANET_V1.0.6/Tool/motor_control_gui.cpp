/**
 * @file    motor_control_gui.cpp
 * @brief   电机控制上位机GUI实现
 */
#include "motor_control_gui.h"
#include "ele_motor_drive.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QComboBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QTextEdit>
#include <QMessageBox>
#include <iomanip>
#include <sstream>

MotorControlGUI::MotorControlGUI(QWidget *parent)
    : QMainWindow(parent), currentCanPort(0), currentMotorId(1) {
    setWindowTitle("电机控制上位机 - CAN指令计算工具");
    setGeometry(100, 100, 1200, 800);

    initUI();
}

MotorControlGUI::~MotorControlGUI() {
}

void MotorControlGUI::initUI() {
    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);

    // 创建各个功能组
    createMotorSelectionGroup();
    createMotorControlGroup();
    createControlModeGroup();
    createParameterGroup();
    createParameterRWGroup();
    createResultGroup();

    // 添加到主布局
    mainLayout->addWidget(new QLabel("=== 电机选择 ==="));
    mainLayout->addWidget(canPortCombo->parentWidget());

    mainLayout->addWidget(new QLabel("=== 电机控制 ==="));
    mainLayout->addWidget(enableBtn->parentWidget());

    mainLayout->addWidget(new QLabel("=== 控制模式 ==="));
    mainLayout->addWidget(impedanceRadio->parentWidget());

    mainLayout->addWidget(new QLabel("=== 参数设置 ==="));
    mainLayout->addWidget(impedanceGroup);
    mainLayout->addWidget(speedGroup);
    mainLayout->addWidget(positionGroup);

    mainLayout->addWidget(new QLabel("=== 参数读写 ==="));
    mainLayout->addWidget(paramTypeCombo->parentWidget());

    mainLayout->addWidget(new QLabel("=== 结果显示 ==="));
    mainLayout->addWidget(resultTextEdit);

    mainLayout->addStretch();
}

void MotorControlGUI::createMotorSelectionGroup() {
    QWidget *selectionWidget = new QWidget();
    QHBoxLayout *layout = new QHBoxLayout(selectionWidget);

    layout->addWidget(new QLabel("CAN口:"));
    canPortCombo = new QComboBox();
    canPortCombo->addItem("CAN0", 0);
    canPortCombo->addItem("CAN1", 1);
    canPortCombo->addItem("CAN2", 2);
    canPortCombo->addItem("CAN3", 3);
    layout->addWidget(canPortCombo);

    layout->addWidget(new QLabel("电机ID:"));
    motorIdSpinBox = new QSpinBox();
    motorIdSpinBox->setMinimum(1);
    motorIdSpinBox->setMaximum(3);
    motorIdSpinBox->setValue(1);
    layout->addWidget(motorIdSpinBox);

    layout->addWidget(new QLabel("当前选择:"));
    selectedMotorLabel = new QLabel("CAN0 - Motor1");
    layout->addWidget(selectedMotorLabel);

    layout->addStretch();

    connect(canPortCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MotorControlGUI::onCanPortChanged);
    connect(motorIdSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MotorControlGUI::onMotorIdChanged);

    canPortCombo->setParent(selectionWidget);
}

void MotorControlGUI::createMotorControlGroup() {
    QWidget *controlWidget = new QWidget();
    QHBoxLayout *layout = new QHBoxLayout(controlWidget);

    enableBtn = new QPushButton("使能");
    disableBtn = new QPushButton("失能");
    setZeroBtn = new QPushButton("置零");
    clearErrorBtn = new QPushButton("清错误");

    layout->addWidget(enableBtn);
    layout->addWidget(disableBtn);
    layout->addWidget(setZeroBtn);
    layout->addWidget(clearErrorBtn);
    layout->addStretch();

    connect(enableBtn, &QPushButton::clicked, this, &MotorControlGUI::onEnableMotor);
    connect(disableBtn, &QPushButton::clicked, this, &MotorControlGUI::onDisableMotor);
    connect(setZeroBtn, &QPushButton::clicked, this, &MotorControlGUI::onSetZero);
    connect(clearErrorBtn, &QPushButton::clicked, this, &MotorControlGUI::onClearError);

    enableBtn->setParent(controlWidget);
}

void MotorControlGUI::createControlModeGroup() {
    QWidget *modeWidget = new QWidget();
    QHBoxLayout *layout = new QHBoxLayout(modeWidget);

    modeGroup = new QButtonGroup();

    impedanceRadio = new QRadioButton("阻抗模式");
    speedRadio = new QRadioButton("速度模式");
    positionRadio = new QRadioButton("位置模式");

    impedanceRadio->setChecked(true);

    modeGroup->addButton(impedanceRadio, 0);
    modeGroup->addButton(speedRadio, 1);
    modeGroup->addButton(positionRadio, 2);

    layout->addWidget(impedanceRadio);
    layout->addWidget(speedRadio);
    layout->addWidget(positionRadio);

    setModeBtn = new QPushButton("设置模式");
    layout->addWidget(setModeBtn);
    layout->addStretch();

    connect(modeGroup, QOverload<int>::of(&QButtonGroup::buttonClicked),
            this, &MotorControlGUI::onControlModeChanged);
    connect(setModeBtn, &QPushButton::clicked, this, &MotorControlGUI::onSetControlMode);

    impedanceRadio->setParent(modeWidget);
}

void MotorControlGUI::createParameterGroup() {
    // 阻抗模式参数
    impedanceGroup = new QGroupBox("阻抗模式参数");
    QGridLayout *impedanceLayout = new QGridLayout(impedanceGroup);

    impedanceLayout->addWidget(new QLabel("位置 (rad):"), 0, 0);
    impedancePosSpinBox = new QDoubleSpinBox();
    impedancePosSpinBox->setRange(-12.5, 12.5);
    impedancePosSpinBox->setValue(0.0);
    impedanceLayout->addWidget(impedancePosSpinBox, 0, 1);

    impedanceLayout->addWidget(new QLabel("速度 (rad/s):"), 0, 2);
    impedanceVelSpinBox = new QDoubleSpinBox();
    impedanceVelSpinBox->setRange(-14.0, 14.0);
    impedanceVelSpinBox->setValue(0.0);
    impedanceLayout->addWidget(impedanceVelSpinBox, 0, 3);

    impedanceLayout->addWidget(new QLabel("Kp:"), 1, 0);
    impedanceKpSpinBox = new QDoubleSpinBox();
    impedanceKpSpinBox->setRange(0, 500);
    impedanceKpSpinBox->setValue(10.0);
    impedanceLayout->addWidget(impedanceKpSpinBox, 1, 1);

    impedanceLayout->addWidget(new QLabel("Kd:"), 1, 2);
    impedanceKdSpinBox = new QDoubleSpinBox();
    impedanceKdSpinBox->setRange(0, 100);
    impedanceKdSpinBox->setValue(1.0);
    impedanceLayout->addWidget(impedanceKdSpinBox, 1, 3);

    impedanceLayout->addWidget(new QLabel("扭矩 (Nm):"), 2, 0);
    impedanceTorqueSpinBox = new QDoubleSpinBox();
    impedanceTorqueSpinBox->setRange(-10.0, 10.0);
    impedanceTorqueSpinBox->setValue(0.0);
    impedanceLayout->addWidget(impedanceTorqueSpinBox, 2, 1);

    impedanceCalcBtn = new QPushButton("计算阻抗指令");
    impedanceLayout->addWidget(impedanceCalcBtn, 2, 2, 1, 2);

    connect(impedanceCalcBtn, &QPushButton::clicked, this, &MotorControlGUI::onCalculateImpedanceCommand);

    // 速度模式参数
    speedGroup = new QGroupBox("速度模式参数");
    speedGroup->setVisible(false);
    QGridLayout *speedLayout = new QGridLayout(speedGroup);

    speedLayout->addWidget(new QLabel("速度 (rad/s):"), 0, 0);
    speedVelSpinBox = new QDoubleSpinBox();
    speedVelSpinBox->setRange(-14.0, 14.0);
    speedVelSpinBox->setValue(0.0);
    speedLayout->addWidget(speedVelSpinBox, 0, 1);

    speedLayout->addWidget(new QLabel("Kp:"), 0, 2);
    speedKpSpinBox = new QDoubleSpinBox();
    speedKpSpinBox->setRange(0, 500);
    speedKpSpinBox->setValue(10.0);
    speedLayout->addWidget(speedKpSpinBox, 0, 3);

    speedLayout->addWidget(new QLabel("Ki:"), 1, 0);
    speedKiSpinBox = new QDoubleSpinBox();
    speedKiSpinBox->setRange(0, 10000);
    speedKiSpinBox->setValue(0.0);
    speedLayout->addWidget(speedKiSpinBox, 1, 1);

    speedCalcBtn = new QPushButton("计算速度指令");
    speedLayout->addWidget(speedCalcBtn, 1, 2, 1, 2);

    connect(speedCalcBtn, &QPushButton::clicked, this, &MotorControlGUI::onCalculateSpeedCommand);

    // 位置模式参数
    positionGroup = new QGroupBox("位置模式参数");
    positionGroup->setVisible(false);
    QGridLayout *positionLayout = new QGridLayout(positionGroup);

    positionLayout->addWidget(new QLabel("位置 (rad):"), 0, 0);
    positionPosSpinBox = new QDoubleSpinBox();
    positionPosSpinBox->setRange(-12.5, 12.5);
    positionPosSpinBox->setValue(0.0);
    positionLayout->addWidget(positionPosSpinBox, 0, 1);

    positionLayout->addWidget(new QLabel("Kvp:"), 0, 2);
    positionKvpSpinBox = new QDoubleSpinBox();
    positionKvpSpinBox->setRange(0, 500);
    positionKvpSpinBox->setValue(10.0);
    positionLayout->addWidget(positionKvpSpinBox, 0, 3);

    positionLayout->addWidget(new QLabel("Kp:"), 1, 0);
    positionKpSpinBox = new QDoubleSpinBox();
    positionKpSpinBox->setRange(0, 500);
    positionKpSpinBox->setValue(10.0);
    positionLayout->addWidget(positionKpSpinBox, 1, 1);

    positionLayout->addWidget(new QLabel("Kd:"), 1, 2);
    positionKdSpinBox = new QDoubleSpinBox();
    positionKdSpinBox->setRange(0, 100);
    positionKdSpinBox->setValue(1.0);
    positionLayout->addWidget(positionKdSpinBox, 1, 3);

    positionLayout->addWidget(new QLabel("Kvi:"), 2, 0);
    positionKviSpinBox = new QDoubleSpinBox();
    positionKviSpinBox->setRange(0, 10000);
    positionKviSpinBox->setValue(0.0);
    positionLayout->addWidget(positionKviSpinBox, 2, 1);

    positionCalcBtn = new QPushButton("计算位置指令");
    positionLayout->addWidget(positionCalcBtn, 2, 2, 1, 2);

    connect(positionCalcBtn, &QPushButton::clicked, this, &MotorControlGUI::onCalculatePositionCommand);
}

void MotorControlGUI::createParameterRWGroup() {
    QWidget *rwWidget = new QWidget();
    QHBoxLayout *layout = new QHBoxLayout(rwWidget);

    layout->addWidget(new QLabel("参数类型:"));
    paramTypeCombo = new QComboBox();
    paramTypeCombo->addItem("CAN_ID", MOTOR_WR_CAN_ID);
    paramTypeCombo->addItem("Current_Limit", MOTOR_WR_Current_Limit);
    paramTypeCombo->addItem("Max_Angle", MOTOR_WR_Max_Angle);
    paramTypeCombo->addItem("Min_Angle", MOTOR_WR_Min_Angle);
    layout->addWidget(paramTypeCombo);

    layout->addWidget(new QLabel("参数值:"));
    paramValueSpinBox = new QDoubleSpinBox();
    paramValueSpinBox->setRange(-10000, 10000);
    paramValueSpinBox->setValue(0.0);
    layout->addWidget(paramValueSpinBox);

    readParamBtn = new QPushButton("读参数");
    writeParamBtn = new QPushButton("写参数");
    layout->addWidget(readParamBtn);
    layout->addWidget(writeParamBtn);
    layout->addStretch();

    connect(readParamBtn, &QPushButton::clicked, this, &MotorControlGUI::onReadParameter);
    connect(writeParamBtn, &QPushButton::clicked, this, &MotorControlGUI::onWriteParameter);

    paramTypeCombo->setParent(rwWidget);
}

void MotorControlGUI::createResultGroup() {
    resultTextEdit = new QTextEdit();
    resultTextEdit->setReadOnly(true);
    resultTextEdit->setMaximumHeight(200);
}

void MotorControlGUI::onCanPortChanged(int index) {
    currentCanPort = index;
    updateParameterUI();
}

void MotorControlGUI::onMotorIdChanged(int value) {
    currentMotorId = value;
    selectedMotorLabel->setText(QString("CAN%1 - Motor%2").arg(currentCanPort).arg(currentMotorId));
}

void MotorControlGUI::onEnableMotor() {
    uint8_t data[8] = {0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    displayCanFrame(data, 8, "使能指令");
}

void MotorControlGUI::onDisableMotor() {
    uint8_t data[8] = {0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    displayCanFrame(data, 8, "失能指令");
}

void MotorControlGUI::onSetZero() {
    uint8_t data[8] = {0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    displayCanFrame(data, 8, "置零指令");
}

void MotorControlGUI::onClearError() {
    uint8_t data[8] = {0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    displayCanFrame(data, 8, "清错误指令");
}

void MotorControlGUI::onControlModeChanged(int id) {
    impedanceGroup->setVisible(id == 0);
    speedGroup->setVisible(id == 1);
    positionGroup->setVisible(id == 2);
}

void MotorControlGUI::onSetControlMode() {
    // 获取当前选择的模式
    int modeId = modeGroup->checkedId();
    uint8_t modeValue = (uint8_t)modeId;  // 0=阻抗, 1=速度, 2=位置

    // 使用参数指令设置控制模式
    uint8_t data[8];
    MotorParamCodec::Float2Bag((float)modeValue, 1, MOTOR_WR_CONTROL_MODE, currentMotorId, data);

    displayCanFrame(data, 8, "设置控制模式指令");
}

void MotorControlGUI::onCalculateImpedanceCommand() {
    float pos = MotorParamValidator::ValidatePosition(impedancePosSpinBox->value());
    float vel = MotorParamValidator::ValidateVelocity(impedanceVelSpinBox->value());
    float kp = MotorParamValidator::ValidateKp(impedanceKpSpinBox->value());
    float kd = MotorParamValidator::ValidateKd(impedanceKdSpinBox->value());
    float torque = MotorParamValidator::ValidateTorque(impedanceTorqueSpinBox->value());

    // 量化参数（按硬件协议）
    unsigned int posInt = MotorParamCodec::FloatToUint(pos, P_MIN, P_MAX, 15);
    unsigned int velInt = MotorParamCodec::FloatToUint(vel, V_MIN, V_MAX, 12);
    unsigned int kpInt = MotorParamCodec::FloatToUint(kp, KP_MIN, KP_MAX, 12);  // 12位
    unsigned int kdInt = MotorParamCodec::FloatToUint(kd, KD_MIN, KD_MAX, 12);  // 12位
    unsigned int torqueInt = MotorParamCodec::FloatToUint(torque, T_MIN, T_MAX, 12);

    // 按硬件协议打包CAN帧
    uint8_t data[8];
    data[0] = (uint8_t)((posInt >> 8) & 0x7F);
    data[1] = (uint8_t)(posInt & 0xFF);
    data[2] = (uint8_t)(velInt >> 4);
    data[3] = (uint8_t)(((velInt & 0xF) << 4) | (kpInt >> 8));
    data[4] = (uint8_t)(kpInt & 0xFF);
    data[5] = (uint8_t)(kdInt >> 4);
    data[6] = (uint8_t)(((kdInt & 0xF) << 4) | (torqueInt >> 8));
    data[7] = (uint8_t)(torqueInt & 0xFF);

    displayCanFrame(data, 8, "阻抗控制指令");
}

void MotorControlGUI::onCalculateSpeedCommand() {
    float vel = MotorParamValidator::ValidateVelocity(speedVelSpinBox->value());
    float kp = MotorParamValidator::ValidateKp(speedKpSpinBox->value());
    float ki = MotorParamValidator::ValidateKi(speedKiSpinBox->value());

    unsigned int velInt = MotorParamCodec::FloatToUint(vel, V_MIN, V_MAX, 12);
    unsigned int kpInt = MotorParamCodec::FloatToUint(kp, KP_MIN, KP_MAX, 9);
    unsigned int kiInt = MotorParamCodec::FloatToUint(ki, KI_MIN, KI_MAX, 14);

    uint8_t data[8];
    data[0] = (velInt >> 8) & 0xFF;
    data[1] = velInt & 0xFF;
    data[2] = (kpInt >> 4) & 0xFF;
    data[3] = ((kpInt & 0x0F) << 4) | ((kiInt >> 10) & 0x0F);
    data[4] = (kiInt >> 2) & 0xFF;
    data[5] = ((kiInt & 0x03) << 6);
    data[6] = 0;
    data[7] = 0;

    displayCanFrame(data, 8, "速度控制指令");
}

void MotorControlGUI::onCalculatePositionCommand() {
    float pos = MotorParamValidator::ValidatePosition(positionPosSpinBox->value());
    float kvp = MotorParamValidator::ValidateKp(positionKvpSpinBox->value());
    float kp = MotorParamValidator::ValidateKp(positionKpSpinBox->value());
    float kd = MotorParamValidator::ValidateKd(positionKdSpinBox->value());
    float kvi = MotorParamValidator::ValidateKi(positionKviSpinBox->value());

    unsigned int posInt = MotorParamCodec::FloatToUint(pos, P_MIN, P_MAX, 15);
    unsigned int kvpInt = MotorParamCodec::FloatToUint(kvp, KP_MIN, KP_MAX, 9);
    unsigned int kpInt = MotorParamCodec::FloatToUint(kp, KP_MIN, KP_MAX, 9);
    unsigned int kdInt = MotorParamCodec::FloatToUint(kd, KD_MIN, KD_MAX, 8);
    unsigned int kviInt = MotorParamCodec::FloatToUint(kvi, KI_MIN, KI_MAX, 14);

    uint8_t data[8];
    data[0] = (posInt >> 8) & 0xFF;
    data[1] = posInt & 0xFF;
    data[2] = (kvpInt >> 4) & 0xFF;
    data[3] = ((kvpInt & 0x0F) << 4) | ((kpInt >> 5) & 0x0F);
    data[4] = ((kpInt & 0x1F) << 3) | ((kdInt >> 5) & 0x07);
    data[5] = ((kdInt & 0x1F) << 3) | ((kviInt >> 11) & 0x07);
    data[6] = (kviInt >> 3) & 0xFF;
    data[7] = ((kviInt & 0x07) << 5);

    displayCanFrame(data, 8, "位置控制指令");
}

void MotorControlGUI::onReadParameter() {
    uint8_t paramType = paramTypeCombo->currentData().toUInt();
    uint8_t data[8];
    MotorParamCodec::Float2Bag(0.0f, 0, paramType, currentMotorId, data);
    displayCanFrame(data, 8, "读参数指令");
}

void MotorControlGUI::onWriteParameter() {
    uint8_t paramType = paramTypeCombo->currentData().toUInt();
    float value = paramValueSpinBox->value();
    uint8_t data[8];
    MotorParamCodec::Float2Bag(value, 1, paramType, currentMotorId, data);
    displayCanFrame(data, 8, "写参数指令");
}

void MotorControlGUI::displayCanFrame(const uint8_t* data, int length, const QString& title) {
    std::stringstream ss;
    ss << title.toStdString() << "\n";
    ss << "CAN ID: 0x" << std::hex << std::setfill('0') << std::setw(3) << (int)currentMotorId << "\n";
    ss << "Data: ";
    for (int i = 0; i < length; ++i) {
        ss << std::hex << std::setfill('0') << std::setw(2) << (int)data[i];
        if (i < length - 1) ss << " ";
    }
    ss << "\n\n";

    resultTextEdit->append(QString::fromStdString(ss.str()));
}

void MotorControlGUI::clearResults() {
    resultTextEdit->clear();
}

void MotorControlGUI::updateParameterUI() {
    selectedMotorLabel->setText(QString("CAN%1 - Motor%2").arg(currentCanPort).arg(currentMotorId));
}

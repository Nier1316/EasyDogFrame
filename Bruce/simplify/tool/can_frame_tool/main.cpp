// 电机 CAN 帧编解码 GUI 工具 (Qt5 Widgets)
// 独立程序，仅依赖同目录 motor_codec.h / motor_decode.h，不碰主工程。
#include <QApplication>
#include <QWidget>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QComboBox>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QString>
#include <QStringList>
#include <vector>
#include "motor_codec.h"
#include "motor_decode.h"

using namespace motorcodec;

// 各模式下 5 个参数的标签（"" 表示该位无用）
static const char* PARAM_LABELS[3][5] = {
    {"期望角度 (rad)", "期望角速度 (rad/s)", "刚度 kp", "阻尼 kd", "前馈扭矩 (Nm)"},
    {"期望角速度 (rad/s)", "速度环 kvp", "(未用)", "(未用)", "速度环 kvi"},
    {"期望角度 (rad)", "位置环 kvp", "速度环 kp", "位置环 kd", "速度环 kvi"},
};
static const char* MODE_NAMES[3] = {"阻抗 IMPEDANCE", "速度 SPEED", "位置 POSITION"};

class MainWindow : public QWidget {
    Q_OBJECT
public:
    MainWindow() {
        setWindowTitle("电机 CAN 帧编解码工具");
        resize(640, 560);
        auto* tabs = new QTabWidget(this);
        tabs->addTab(buildEncodeTab(), "参数 → CAN 帧");
        tabs->addTab(buildDecodeTab(), "CAN 帧 → 含义");
        auto* lay = new QVBoxLayout(this);
        lay->addWidget(tabs);
        refreshLabels(0);
    }

private:
    QComboBox* encMode_ = nullptr;
    QLabel* paramLbl_[5] = {};
    QLineEdit* paramEdit_[5] = {};
    QPlainTextEdit* encOut_ = nullptr;
    QLineEdit* decInput_ = nullptr;
    QComboBox* decMode_ = nullptr;
    QPlainTextEdit* decOut_ = nullptr;

    QWidget* buildEncodeTab();
    QWidget* buildDecodeTab();

private slots:
    void refreshLabels(int modeIdx);
    void doEncode();
    void doDecode();
};

QWidget* MainWindow::buildEncodeTab() {
    auto* w = new QWidget;
    auto* lay = new QVBoxLayout(w);

    auto* form = new QFormLayout;
    encMode_ = new QComboBox;
    for (auto* n : MODE_NAMES) encMode_->addItem(n);
    form->addRow("控制模式", encMode_);
    for (int i = 0; i < 5; i++) {
        paramLbl_[i] = new QLabel;
        paramEdit_[i] = new QLineEdit("0");
        form->addRow(paramLbl_[i], paramEdit_[i]);
    }
    lay->addLayout(form);

    auto* btn = new QPushButton("生成 CAN 帧");
    lay->addWidget(btn);

    encOut_ = new QPlainTextEdit;
    encOut_->setReadOnly(true);
    encOut_->setStyleSheet("font-family: monospace;");
    lay->addWidget(encOut_, 1);

    connect(encMode_, SIGNAL(currentIndexChanged(int)), this, SLOT(refreshLabels(int)));
    connect(btn, SIGNAL(clicked()), this, SLOT(doEncode()));
    return w;
}

void MainWindow::refreshLabels(int modeIdx) {
    if (modeIdx < 0 || modeIdx > 2) return;
    for (int i = 0; i < 5; i++) {
        const char* t = PARAM_LABELS[modeIdx][i];
        paramLbl_[i]->setText(QString("p%1  %2").arg(i + 1).arg(t));
        bool unused = (QString(t) == "(未用)");
        paramEdit_[i]->setEnabled(!unused);
        if (unused) paramEdit_[i]->setText("0");
    }
}

void MainWindow::doEncode() {
    int m = encMode_->currentIndex();
    float p[5];
    for (int i = 0; i < 5; i++) p[i] = paramEdit_[i]->text().toFloat();

    bool clamped = false;
    Frame f = encode((Mode)m, p[0], p[1], p[2], p[3], p[4], &clamped);

    QString out;
    out += QString("模式: %1\n\n").arg(MODE_NAMES[m]);
    out += "CAN 数据 (8 字节):\n";
    QStringList hex, dec;
    for (int i = 0; i < 8; i++) {
        hex << QString::asprintf("0x%02X", f[i]);
        dec << QString::number(f[i]);
    }
    out += "  HEX:  " + hex.join(" ") + "\n";
    out += "  DEC:  " + dec.join(" ") + "\n\n";
    out += "逐字节:\n";
    for (int i = 0; i < 8; i++)
        out += QString::asprintf("  data[%d] = 0x%02X (%3d)\n", i, f[i], f[i]);
    if (clamped)
        out += "\n⚠ 有参数超出量程，已被钳位到边界后再编码。";
    encOut_->setPlainText(out);
}

QWidget* MainWindow::buildDecodeTab() {
    auto* w = new QWidget;
    auto* lay = new QVBoxLayout(w);

    lay->addWidget(new QLabel(
        "输入 8 字节（十六进制或十进制，空格/逗号分隔）\n"
        "例: 0xFF 0x7F 0xF0 0x00 0x00 0x00 0x07 0xFF"));
    decInput_ = new QLineEdit;
    lay->addWidget(decInput_);

    auto* row = new QHBoxLayout;
    row->addWidget(new QLabel("按模式解读:"));
    decMode_ = new QComboBox;
    decMode_->addItem("全部三种", -1);
    decMode_->addItem(MODE_NAMES[0], 0);
    decMode_->addItem(MODE_NAMES[1], 1);
    decMode_->addItem(MODE_NAMES[2], 2);
    row->addWidget(decMode_, 1);
    auto* btn = new QPushButton("解析");
    row->addWidget(btn);
    lay->addLayout(row);

    decOut_ = new QPlainTextEdit;
    decOut_->setReadOnly(true);
    decOut_->setStyleSheet("font-family: monospace;");
    lay->addWidget(decOut_, 1);

    connect(btn, SIGNAL(clicked()), this, SLOT(doDecode()));
    connect(decInput_, SIGNAL(returnPressed()), this, SLOT(doDecode()));
    return w;
}

void MainWindow::doDecode() {
    QString raw = decInput_->text();
    raw.replace(',', ' ');
    const QStringList toks = raw.split(' ', Qt::SkipEmptyParts);

    std::vector<uint8_t> bytes;
    bool ok = true;
    for (const QString& tk : toks) {
        bool o;
        int v = tk.startsWith("0x", Qt::CaseInsensitive)
                    ? tk.mid(2).toInt(&o, 16)
                    : tk.toInt(&o, 10);
        if (!o || v < 0 || v > 255) { ok = false; break; }
        bytes.push_back((uint8_t)v);
    }

    if (!ok) { decOut_->setPlainText("❌ 解析失败：含非法字节（应为 0~255 / 0x00~0xFF）。"); return; }

    QString note;
    if (bytes.size() < 8) {
        note = QString("⚠ 只给了 %1 字节，CAN 帧应为 8 字节；不足部分按 0x00 补齐。\n"
                       "  (如从 sendcan.csv 复制，注意别漏了第 1 列 d0)\n\n").arg(bytes.size());
        bytes.resize(8, 0);
    } else if (bytes.size() > 8) {
        note = QString("⚠ 给了 %1 字节，只取前 8 个。\n\n").arg(bytes.size());
        bytes.resize(8);
    }

    QStringList hex;
    for (int i = 0; i < 8; i++) hex << QString::asprintf("0x%02X", bytes[i]);
    QString out = note + "输入帧: " + hex.join(" ") + "\n\n";
    out += QString::fromStdString(decode(bytes.data(), decMode_->currentData().toInt()));
    decOut_->setPlainText(out);
}

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    MainWindow w;
    w.show();
    return app.exec();
}

#include "main.moc"

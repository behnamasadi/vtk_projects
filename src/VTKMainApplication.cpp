#include "CustomVTKItem.h"
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QtQml>

int main(int argc, char *argv[]) {
  QGuiApplication app(argc, argv);

  qmlRegisterType<CustomVTKItem>("VTK", 1, 0, "VTKItem");

  QQmlApplicationEngine engine;
  engine.load(QUrl(QStringLiteral("qrc:/main.qml")));

  return app.exec();
}

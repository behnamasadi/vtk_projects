// main.cpp

#include "VTKBackend.hpp"
#include <QGuiApplication>
#include <QQmlApplicationEngine>

vtkStandardNewMacro(VTKBackend::Data);

int main(int argc, char *argv[]) {
  QQuickVTKItem::setGraphicsApi();

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

  QGuiApplication app(argc, argv);

  qmlRegisterType<VTKBackend>("VTKBackend", 1, 0, "VTKBackend");

  QQmlApplicationEngine engine;

  engine.addImportPath("/home/behnam/usr/lib/qml");
  engine.load(QUrl(QStringLiteral("qrc:/qml/main.qml")));

  if (engine.rootObjects().isEmpty())
    return -1;

  return app.exec();
}

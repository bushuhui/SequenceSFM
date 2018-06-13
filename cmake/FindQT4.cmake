find_package(Qt4)
if(QT4_FOUND)
  set(QT4_INCLUDES ${QT_INCLUDES})
  set(QT4_DEFINITIONS ${QT_DEFINITIONS})
  set(QT4_LIBRARIES Qt4::QtCore Qt4::QtGui Qt4::QtNetwork Qt4::QtOpenGL Qt4::QtSql Qt4::QtSvg Qt4::QtXml)

  set(QT_DEFINITIONS ${QT4_DEFINITIONS})
  set(QT_LIBRARIES ${QT4_LIBRARIES})
endif()



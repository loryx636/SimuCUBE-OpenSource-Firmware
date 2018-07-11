TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    hid.c \
    main.cpp

HEADERS += \
    hidapi.h \
    ../../Inc/config_comm_defines.h

LIBS += -lSetupAPI

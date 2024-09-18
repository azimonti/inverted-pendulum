/************************/
/*      main1.cpp       */
/*    Version 1.0       */
/*     2023/04/13       */
/*  © Marco Azimonti    */
/************************/

#include "log/log.h"
#include "gl_app.h"

int main(int, char**)
{
    bool bFileLog = false;
    LOGGER_PARAM(logging::LEVELMAX, logging::INFO);
    LOGGER_PARAM(logging::LOGTIME, true);
    if (bFileLog)
    {
        LOGGER_PARAM(logging::FILENAME, "out_viewer.log");
        LOGGER_PARAM(logging::FILEOUT, true);
    }
    GLApp app;
    app.onInit();
    app.mainLoop();
    return 0;
}

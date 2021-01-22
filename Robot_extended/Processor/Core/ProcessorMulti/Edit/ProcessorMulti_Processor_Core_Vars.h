//You need to modify this file.

#ifndef PROCESSORMULTI_PROCESSOR_CORE_VARS_H
#define PROCESSORMULTI_PROCESSOR_CORE_VARS_H

#include<RobotSDK_Global.h>

/*! \defgroup ProcessorMulti_Processor_Core_Vars ProcessorMulti_Processor_Core_Vars
	\ingroup ProcessorMulti_Processor_Core
	\brief ProcessorMulti_Processor_Core_Vars defines the Vars in ProcessorMulti_Processor_Core.
*/

/*! \addtogroup ProcessorMulti_Processor_Core_Vars
	@{
*/

/*! \file ProcessorMulti_Processor_Core_Vars.h
	 Defines the Vars of ProcessorMulti_Processor_Core
*/

//*******************Please add other headers below*******************


#include "ProcessorMulti_Processor_Core_ParamsData.h"

//The Vars is defined as below
/*! \class ProcessorMulti_Processor_Core_Vars 
	\brief The Vars of ProcessorMulti_Processor_Core.
	\details **Please add details below**

*/
class ROBOTSDK_OUTPUT ProcessorMulti_Processor_Core_Vars 
{
public:
	/*! \fn ProcessorMulti_Processor_Core_Vars()
		\brief The constructor of ProcessorMulti_Processor_Core_Vars. [required]
		\details ****Please add details below****

	*/
	ProcessorMulti_Processor_Core_Vars() 
	{
        mapWidth = 600;
        mapHeight = 600;

        if(mapWidth > 0 && mapHeight > 0)
        {
            map = new int* [mapHeight];
            for(int i = 0 ; i < mapHeight ; i ++)
                map[i] = new int [mapWidth];
            for(int i=0; i<mapHeight; i++)
                for(int j=0; j<mapWidth; j++)
                    map[i][j] = 128;
        }
        else map = NULL;
	}
	/*! \fn ~ProcessorMulti_Processor_Core_Vars()
		\brief The destructor of ProcessorMulti_Processor_Core_Vars. [required]
		\details *****Please add details below*****

	*/
	~ProcessorMulti_Processor_Core_Vars() 
	{
        if(map != NULL)
        {
            for(int i = 0 ; i < mapHeight ; i ++)
            {
                delete map[i];
            }
            delete [] map;
            map = NULL;
        }
	}
public:
	//*******************Please add variables below*******************
    int** map;
    int mapHeight;
    int mapWidth;
};

/*! @}*/ 

#endif

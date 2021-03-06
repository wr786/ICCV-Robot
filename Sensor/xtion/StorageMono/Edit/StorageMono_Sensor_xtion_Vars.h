//You need to modify this file.

#ifndef STORAGEMONO_SENSOR_XTION_VARS_H
#define STORAGEMONO_SENSOR_XTION_VARS_H

#include<RobotSDK_Global.h>

/*! \defgroup StorageMono_Sensor_xtion_Vars StorageMono_Sensor_xtion_Vars
	\ingroup StorageMono_Sensor_xtion
	\brief StorageMono_Sensor_xtion_Vars defines the Vars in StorageMono_Sensor_xtion.
*/

/*! \addtogroup StorageMono_Sensor_xtion_Vars
	@{
*/

/*! \file StorageMono_Sensor_xtion_Vars.h
	 Defines the Vars of StorageMono_Sensor_xtion
*/

//*******************Please add other headers below*******************


#include "StorageMono_Sensor_xtion_ParamsData.h"

//The Vars is defined as below
/*! \class StorageMono_Sensor_xtion_Vars 
	\brief The Vars of StorageMono_Sensor_xtion.
	\details **Please add details below**

*/
class ROBOTSDK_OUTPUT StorageMono_Sensor_xtion_Vars 
{
public:
	/*! \fn StorageMono_Sensor_xtion_Vars()
		\brief The constructor of StorageMono_Sensor_xtion_Vars. [required]
		\details ****Please add details below****

	*/
	StorageMono_Sensor_xtion_Vars() 
	{
		
	}
	/*! \fn ~StorageMono_Sensor_xtion_Vars()
		\brief The destructor of StorageMono_Sensor_xtion_Vars. [required]
		\details *****Please add details below*****

	*/
	~StorageMono_Sensor_xtion_Vars() 
	{
		
	}
public:
	//*******************Please add variables below*******************
    QFile timestampWriter;
    QString colorDir, depthDir;
    int frameNum;
};

/*! @}*/ 

#endif

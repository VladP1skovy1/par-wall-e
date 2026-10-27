#ifndef IMR_SND_H
#define IMR_SND_H

#include "gap_and_valley.h"
#include "robot_client/robot_client.h"

class SNDNavigation {
	public: 
		SNDNavigation(CRobotClient &robot);
		bool isRisingGapSafe(Gap* pRisingGap, int iValleyDir, std::vector<double> fullLP, double fScanRes, double fMaxRange, double R);
		bool isFilterClear(int iCenterSector, double width, double forwardLength, bool bDoRearCheck, std::vector<double> fullLP, double angRes, bool bPrint);
		void GoTo(double aX, double aY, double aYaw, bool slowDown);
      
	private:
		CRobotClient &robot;
		double R; 				// The radius of the minimum circle which contains the robot
		double minGapWidth; 	// Minimum passage width the driver will try to exploit
		double safetyDistMax; 	// Maximum distance allowed from an obstacle
		double maxSpeed; 		// Maximum speed allowed
		double maxTurnRate; 	// Maximum angular speed allowed
		double goalPositionTol; // Maximum distance allowed from the final goal for the algorithm to stop. (m)
		double goalAngleTol; 	// Maximum angular error from the final goal position for the algorithm to stop (rad)
		double fMaxRange;
		double fScanRes;
		int iNumLPs;
		int iNumSectors;
    
		// Position2dProxy *odo;
		// Position2dProxy *global;
		// LaserProxy *laser;
    
		bool init();

};
#endif //GAP_ND_NAV_H_

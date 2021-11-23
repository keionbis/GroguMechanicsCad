import com.neuronrobotics.sdk.common.BowlerAbstractDevice

import com.neuronrobotics.bowlerstudio.creature.ICadGenerator;
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.bowlerstudio.BowlerStudio
import com.neuronrobotics.bowlerstudio.creature.CreatureLab;
import org.apache.commons.io.IOUtils;
import org.apache.http.client.params.AllClientPNames
import org.eclipse.jetty.server.handler.MovedContextHandler

import com.neuronrobotics.bowlerstudio.vitamins.*;
import com.neuronrobotics.sdk.addons.kinematics.AbstractLink
import com.neuronrobotics.sdk.addons.kinematics.DHLink
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.LinkConfiguration
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.IDeviceAddedListener
import com.neuronrobotics.sdk.common.IDeviceConnectionEventListener

import java.nio.file.Paths;

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.Cylinder
import eu.mihosoft.vrl.v3d.FileUtil
import eu.mihosoft.vrl.v3d.Parabola
import eu.mihosoft.vrl.v3d.RoundedCube
import eu.mihosoft.vrl.v3d.RoundedCylinder
import eu.mihosoft.vrl.v3d.Sphere
import eu.mihosoft.vrl.v3d.Transform

import javafx.scene.transform.Affine;
import  eu.mihosoft.vrl.v3d.ext.quickhull3d.*
import eu.mihosoft.vrl.v3d.parametrics.LengthParameter
import eu.mihosoft.vrl.v3d.Vector3d


double grid =36
double cornerOffset=grid*1.75
double boardx=8.5*25.4+cornerOffset
double boardy=11.0*25.4+cornerOffset
// Scoot arm over so the paper doesn't awkwardly hang out over edge
double cornerNudge = -10
// radius of rounded corners on base plate
double cornerRadius=5;

CSG reverseDHValues(CSG incoming,DHLink dh ){
	//println "Reversing "+dh
	TransformNR step = new TransformNR(dh.DhStep(0))
	Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
	return incoming.transformed(move)
}

CSG moveDHValues(CSG incoming,DHLink dh ){
	TransformNR step = new TransformNR(dh.DhStep(0)).inverse()
	Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
	return incoming.transformed(move)
}

return new ICadGenerator(){

    int bracketOneKeepawayDistance = 50

	double motorGearPlateThickness = 0
	double boardThickness = 10
	
	def thrustBearingSize = "Thrust_1andAHalfinch"
	double radiusOfGraspingObject=12.5;
	
	double thrustBearing_inset_Into_bottom = 1
	double topOfHornToBotomOfBaseLinkDistance = movingPartClearence-thrustBearing_inset_Into_bottom
	
	double springboltRotation=22
	double GripperServoYOffset = 35
	
	double springRadius=35

	
	def cornerRad=2
	String boltsize = "M5x25"
	def insert=["heatedThreadedInsert", "M5"]
	def insertCamera=["heatedThreadedInsert", "M5"]
	def insertMeasurments= Vitamins.getConfiguration(insert[0],
		insert[1])
	double cameraInsertLength = insertMeasurments.installLength
	
	HashMap<String,Object> measurmentsMotor = Vitamins.getConfiguration(  "LewanSoulMotor","lx_224")
	HashMap<String,Object> measurmentsHorn = Vitamins.getConfiguration(  measurmentsMotor.shaftType,measurmentsMotor.shaftSize)
	
	double motorz =  measurmentsMotor.body_z
	double motorPassiveLinkSideWasherTHickness=measurmentsMotor.shoulderHeight
	double hornKeepawayLen = measurmentsHorn.mountPlateToHornTop
	double hornDiameter = measurmentsHorn.hornDiameter
	double centerTheMotorsValue=motorz/2;
	double linkYDimention = measurmentsMotor.body_x;
	double movingPartClearence =motorPassiveLinkSideWasherTHickness
	double totalMotorAndHorn = motorz+hornKeepawayLen+movingPartClearence;
	
	
	
	double linkThickness = hornKeepawayLen
	double grooveDepth=linkThickness/2
	double centerlineToOuterSurfacePositiveZ = centerTheMotorsValue+hornKeepawayLen
	double centerlineToOuterSurfaceNegativeZ = -(centerTheMotorsValue+movingPartClearence+linkThickness)
	CSG linkBuildingBlockRoundCyl = new Cylinder(linkYDimention/2,linkYDimention/2,linkThickness,30)
		.toCSG()
	CSG linkBuildingBlockRoundSqu = new RoundedCube(linkYDimention,linkYDimention,linkThickness)
		.cornerRadius(cornerRad)
		.toCSG()
		.toZMin()
	CSG linkBuildingBlockRound = new RoundedCylinder(linkYDimention/2,linkThickness)
		.cornerRadius(cornerRad)
		.toCSG()
    CSG cameraBuildingBlockRound = new RoundedCylinder(linkYDimention/2,cameraInsertLength+1)
		.cornerRadius(cornerRad)
		.toCSG()
	CSG linkBuildingBlock = CSG.hullAll([
	linkBuildingBlockRound.movey(linkYDimention),
	linkBuildingBlockRound
	])
	.toZMin()
	.movey(-5)
	LengthParameter offset		= new LengthParameter("printerOffset",0.5,[2,0])
	double offsetValue = 0.6
	@Override
	public ArrayList<CSG> generateCad(DHParameterKinematics d, int linkIndex) {
		System.out.println( "Total motor and horn length "+totalMotorAndHorn)
		offset.setMM(offsetValue)
		def vitaminLocations = new HashMap<TransformNR,ArrayList<String>>()
		ArrayList<DHLink> dhLinks = d.getChain().getLinks()
		ArrayList<CSG> allCad=new ArrayList<>()
		int i=linkIndex;
		DHLink dh = dhLinks.get(linkIndex)
		// Hardware to engineering units configuration
		LinkConfiguration conf = d.getLinkConfiguration(i);
		// Engineering units to kinematics link (limits and hardware type abstraction)
		AbstractLink abstractLink = d.getAbstractLink(i);
		// Transform used by the UI to render the location of the object
		Affine manipulator = dh.getListener();
		// loading the vitamins referenced in the configuration
		//CSG servo=   Vitamins.get(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())
		CSG motorModel=   Vitamins.get(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())
		
	
		
		double zOffset = motorModel.getMaxZ()
		TransformNR locationOfMotorMount = new TransformNR(dh.DhStep(0)).inverse()
		def shaftLocation = locationOfMotorMount.copy()
		def thrustMeasurments= Vitamins.getConfiguration("ballBearing",
			thrustBearingSize)
		def baseCorRad = thrustMeasurments.outerDiameter/2+5
		
		double servoAllignmentAngle=0
		CSG gripperMotor=null;
		TransformNR locationOfGripperHinge=null
		TransformNR locationOfServo=null 
		double servoZOffset = 0;
		def insert=["heatedThreadedInsert", "M5"]
		def insertMeasurments= Vitamins.getConfiguration(insert[0],
			insert[1])
		if(linkIndex==0)
			shaftLocation.translateY(0)
		else if(linkIndex==1) {
			shaftLocation.translateZ(centerTheMotorsValue)
			
		}
		if(linkIndex==0 || linkIndex==1) {
				vitaminLocations.put(shaftLocation, [
					conf.getShaftType(),
					conf.getShaftSize()
			])
		}

		TransformNR locationOfBearing = locationOfMotorMount.copy().translateY(thrustBearing_inset_Into_bottom)
		if(linkIndex==0) {
			vitaminLocations.put(locationOfBearing, [
				"ballBearing",
				thrustBearingSize
			])
		}
		
		CSG vitamin_LewanSoulHorn_round_m3_bolts = Vitamins.get("LewanSoulHorn", "round_m3_bolts")
		double springSupportLength = linkYDimention+linkThickness*2.0+30
		double linkOneSupportWidth=40+linkThickness*2
		if(linkIndex==1) {

			def springBolt = locationOfMotorMount.copy().times(new TransformNR(0,0,0,new RotationNR(0,-springboltRotation,0))).times(new TransformNR()
				.translateZ(centerlineToOuterSurfacePositiveZ+linkThickness).translateY(-springRadius-4))
			
			def springBolt2 = locationOfMotorMount.copy()
			.times(new TransformNR(0,0,0,new RotationNR(0,-springboltRotation,0)))
			.times(new TransformNR()
				.translateZ(-springSupportLength/2-0.5).translateY(-springRadius-4))
			.times(new TransformNR(0,0,0,new RotationNR(180,0,0)))
			
			def mountBoltOne =locationOfMotorMount.copy()
							.times(new TransformNR().translateZ(centerlineToOuterSurfacePositiveZ+linkThickness)
								.translateY(-bracketOneKeepawayDistance))
			def mountBoltTwo=mountBoltOne.copy()
								.times(new TransformNR()
									.translateY(+20))
							
			vitaminLocations.put(mountBoltOne,["capScrew", boltsize])
			vitaminLocations.put(mountBoltOne.times(new TransformNR().translateZ(-linkThickness-insertMeasurments.installLength)),
				insert)
			vitaminLocations.put(mountBoltTwo,["capScrew", boltsize])
			vitaminLocations.put(mountBoltTwo.times(new TransformNR().translateZ(-linkThickness-insertMeasurments.installLength)),
				insert)
			vitaminLocations.put(springBolt,["capScrew", boltsize])
			vitaminLocations.put(springBolt.times(new TransformNR().translateZ(-linkThickness-insertMeasurments.installLength)),
				insert)
			vitaminLocations.put(springBolt2,["capScrew", boltsize])
			vitaminLocations.put(springBolt2.times(new TransformNR().translateZ(-linkThickness-insertMeasurments.installLength)),
				insert)
		}
		if(linkIndex==0) {
			def mountBoltOne =locationOfMotorMount.copy()
								.times(new TransformNR(-springSupportLength+insertMeasurments.diameter+cornerRadius,
									-linkOneSupportWidth/2,
									insertMeasurments.diameter/2+cornerRadius,new RotationNR(90,0,0)))
				
					//.translateY(-centerlineToOuterSurfaceNegativeZ+linkThickness)
				//.translateX(-insertMeasurments.diameter-cornerRadius)
				
			def mountBoltTwo=locationOfMotorMount.copy()
								.times(new TransformNR(-springSupportLength+insertMeasurments.diameter+cornerRadius,
									linkOneSupportWidth/2,
									insertMeasurments.diameter/2+cornerRadius,new RotationNR(-90,0,0)))
			vitaminLocations.put(mountBoltOne.times(new TransformNR().translateZ(5)),["capScrew", boltsize])
			vitaminLocations.put(mountBoltOne.times(new TransformNR().translateZ(-insertMeasurments.installLength)),
				insert)
			vitaminLocations.put(mountBoltTwo.times(new TransformNR().translateZ(5)),["capScrew", boltsize])
			vitaminLocations.put(mountBoltTwo.times(new TransformNR().translateZ(-insertMeasurments.installLength)),
				insert)
		}

		//if(linkIndex==2||linkIndex==1||linkIndex==0 ){
		TransformNR motorLocation = new TransformNR(0,0,centerTheMotorsValue,new RotationNR())
		if(linkIndex==4||linkIndex==3||linkIndex==2||linkIndex==1||linkIndex==0  ){
			LinkConfiguration confPrior = d.getLinkConfiguration(i+1);
			def vitaminType = confPrior.getElectroMechanicalType()
			def vitaminSize = confPrior.getElectroMechanicalSize()
			//println "Adding Motor "+vitaminType
			
			if(linkIndex==0)
				motorLocation=motorLocation.times(new TransformNR(0,0,0,new RotationNR(0,180,0)))
			if(linkIndex==1)
					motorLocation=motorLocation.times(new TransformNR(0,0,0,new RotationNR(0,-90,0)))
			if(linkIndex==2)
				motorLocation=new TransformNR().times(new TransformNR(0,0,0,new RotationNR(0,-90,0)))
			if(linkIndex==4) {
				
					motorLocation=new TransformNR(0,0,d.getDH_D(linkIndex+1)-centerTheMotorsValue,new RotationNR(0,0,0)).times(motorLocation.times(new TransformNR(0,0,d.getDH_D(linkIndex),new RotationNR(0,-90,0))))
					
			}
			if(linkIndex<2)
			vitaminLocations.put(motorLocation, [
				vitaminType,
				vitaminSize
			])

		}
	
		if(linkIndex==2) {
			allCad.addAll(ScriptingEngine.gitScriptRun(
			"https://github.com/Halloween2020TheChild/GroguMechanicsCad.git", // git location of the library
			"link3.groovy" , // file to load
			// Parameters passed to the funcetion
			[d, linkIndex,centerTheMotorsValue,motorLocation]
			))
		}
		if(linkIndex==3) {
			allCad.addAll(ScriptingEngine.gitScriptRun(
			"https://github.com/Halloween2020TheChild/GroguMechanicsCad.git", // git location of the library
			"wrist1.groovy" , // file to load
			// Parameters passed to the funcetion
			[d, linkIndex,centerTheMotorsValue,motorLocation]
			))
		}
		if(linkIndex==4) {
			allCad.addAll(ScriptingEngine.gitScriptRun(
			"https://github.com/Halloween2020TheChild/GroguMechanicsCad.git", // git location of the library
			"wrist2.groovy" , // file to load
			// Parameters passed to the funcetion
			[d, linkIndex,centerTheMotorsValue,motorLocation]
			))
		}
		if(linkIndex==5) {
			allCad.addAll(ScriptingEngine.gitScriptRun(
			"https://github.com/Halloween2020TheChild/GroguMechanicsCad.git", // git location of the library
			"wrist3.groovy" , // file to load
			// Parameters passed to the funcetion
			[d, linkIndex,centerTheMotorsValue,motorLocation]
			))
		}
		if(linkIndex==6) {
			allCad.addAll(ScriptingEngine.gitScriptRun(
			"https://github.com/Halloween2020TheChild/GroguMechanicsCad.git", // git location of the library
			"tool.groovy" , // file to load
			// Parameters passed to the funcetion
			[d, linkIndex,centerTheMotorsValue,motorLocation]
			))
		}
		//CSG tmpSrv = moveDHValues(servo,dh)

		//Compute the location of the base of this limb to place objects at the root of the limb
		//TransformNR step = d.getRobotToFiducialTransform()
		//Transform locationOfBaseOfLimb = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)

		double totalMass = 0;
		TransformNR centerOfMassFromCentroid=new TransformNR();
		def vitamins =[]
		for(TransformNR tr: vitaminLocations.keySet()) {
			def vitaminType = vitaminLocations.get(tr)[0]
			def vitaminSize = vitaminLocations.get(tr)[1]

			HashMap<String, Object>  measurments = Vitamins.getConfiguration( vitaminType,vitaminSize)
			offset.setMM(offsetValue)
			CSG vitaminCad=   Vitamins.get(vitaminType,vitaminSize)
			Transform move = TransformFactory.nrToCSG(tr)
			def part = vitaminCad.transformed(move)
			part.setManipulator(manipulator)
			vitamins.add(part)

			def massCentroidYValue = measurments.massCentroidY
			def massCentroidXValue = measurments.massCentroidX
			def massCentroidZValue = measurments.massCentroidZ
			def massKgValue = measurments.massKg
			//println vitaminType+" "+vitaminSize
			TransformNR COMCentroid = tr.times(
					new TransformNR(massCentroidXValue,massCentroidYValue,massCentroidZValue,new RotationNR())
					)
			totalMass+=massKgValue
			//do com calculation here for centerOfMassFromCentroid and totalMass
		}
		//Do additional CAD and add to the running CoM
		conf.setMassKg(totalMass)
		conf.setCenterOfMassFromCentroid(centerOfMassFromCentroid)
		Transform actuatorSpace = TransformFactory.nrToCSG(locationOfMotorMount)
		def tipCupCircle = linkBuildingBlockRound.movez(centerlineToOuterSurfacePositiveZ)
		def gripperLug = linkBuildingBlockRound.movez(centerlineToOuterSurfaceNegativeZ)
		def actuatorCircle = tipCupCircle.transformed(actuatorSpace)
		def actuatorCirclekw = linkBuildingBlockRoundCyl.movez(centerlineToOuterSurfacePositiveZ).transformed(actuatorSpace)
		def passivLinkLug = gripperLug.transformed(actuatorSpace)
		double offsetOfLinks=0.0
		double braceBackSetFromMotorLinkTop=1.0
		if(linkIndex==1) {
			double braceDistance=-hornDiameter/2;
			
			double linkClearence = totalMotorAndHorn/2
			def mountMotorSidekw = linkBuildingBlockRoundCyl
										.movez(centerTheMotorsValue)
										.movex(-linkClearence-movingPartClearence)
			def mountMotorSide = linkBuildingBlockRound
										.movez(centerTheMotorsValue)
										.movex(-linkClearence-movingPartClearence)
			def mountPassiveSide = linkBuildingBlockRoundSqu
										.movez(-centerTheMotorsValue-linkThickness)
										.movex(-linkClearence-movingPartClearence)
		   def mountPassiveSideAlligned = linkBuildingBlockRoundSqu
										.movez(centerlineToOuterSurfaceNegativeZ)
										.movex(-linkClearence-movingPartClearence)
			def clearencelugMotorSide = mountMotorSide.movex(-dh.getR()+bracketOneKeepawayDistance)
			def clearencelugMotorSidekw = mountMotorSidekw.movex(-dh.getR()+bracketOneKeepawayDistance)
			def clearencelugPassiveSide = mountPassiveSide.movex(-dh.getR()+bracketOneKeepawayDistance)
			CSG motorLink = actuatorCircle.movez(-offsetOfLinks).movex(bracketOneKeepawayDistance)
			CSG motorLinkkw = actuatorCirclekw.movez(-offsetOfLinks).movex(bracketOneKeepawayDistance)
			def bracemountPassiveSideAlligned = linkBuildingBlockRound
													.movez(centerlineToOuterSurfacePositiveZ)
													.movez(-offsetOfLinks)
													.movey(braceDistance)
													.movex(-linkClearence-movingPartClearence-dh.getR()+bracketOneKeepawayDistance)
			def bracemountMotorSide=actuatorCircle
									.movez(-braceBackSetFromMotorLinkTop)
									.movex(dh.getR()-hornDiameter*1.5)
									.movey(braceDistance)
										
			def brace = CSG.unionAll([
				bracemountMotorSide,
				bracemountPassiveSideAlligned.movez(-braceBackSetFromMotorLinkTop)
				]).hull()
			brace = brace
						.union([
							brace
							.movez(-centerlineToOuterSurfacePositiveZ+centerlineToOuterSurfaceNegativeZ+offsetOfLinks+braceBackSetFromMotorLinkTop),
							clearencelugMotorSide,clearencelugPassiveSide
							]
						).hull()
			def passiveSide = mountPassiveSideAlligned.union(passivLinkLug).hull()
			def motorSidePlate = CSG.hullAll([clearencelugMotorSide,mountMotorSide]);
			motorSidePlate=CSG.hullAll([motorSidePlate,motorSidePlate.toZMax().movez(centerlineToOuterSurfacePositiveZ-offsetOfLinks)])
			def motorSidePlatekw = CSG.hullAll([clearencelugMotorSidekw,mountMotorSidekw]);
			motorSidePlatekw=CSG.hullAll([motorSidePlatekw,motorSidePlatekw.toZMax().movez(centerlineToOuterSurfacePositiveZ-offsetOfLinks)])
			
			def center = CSG.unionAll([mountPassiveSideAlligned,mountMotorSide,clearencelugMotorSide,clearencelugPassiveSide])
							.hull()
		    CSG motorToCut = Vitamins.get(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())
							.rotz(180)
							.movez(centerTheMotorsValue)
							.transformed(actuatorSpace)
			CSG MotorMountBracketkw = actuatorCirclekw.movez(-offsetOfLinks)
							.union(motorLinkkw)
							.hull()
			//Sprint path section	

			CSG grooveInner = new Cylinder(springRadius,springRadius-grooveDepth/2,linkThickness/2,60).toCSG()
			CSG grooveOuter = new Cylinder(springRadius-grooveDepth/2,springRadius,linkThickness/2,60).toCSG().movez(linkThickness/2)
			CSG springPathCore=grooveInner.union(grooveOuter)
			
			CSG springPathCoreKW=springPathCore.getBoundingBox()
			springPathCore=springPathCore
			double offsetSprings = actuatorCircle.getMaxZ()
			def springPathDriveSideCutout=new Cylinder(springRadius+1,springRadius+1,linkThickness,60).toCSG().toZMax()
								.movez(offsetSprings-0.5)
								.union(new Cylinder(springRadius+1,springRadius+1,linkThickness,60).toCSG().toZMax()
								.movez(-linkOneSupportWidth/2+0.5))
			def springPath=springPathCore.difference(springPathCoreKW.toXMin())
			def radiusOfSpringBoltKW = springRadius/4
			def springMountKW =  new Cylinder(radiusOfSpringBoltKW,linkThickness).toCSG()
									.union(new Cylinder(radiusOfSpringBoltKW,linkThickness).toCSG().movey(-dh.getR()-springRadius))
									.union(new Cylinder(radiusOfSpringBoltKW,linkThickness).toCSG().movex(-springRadius))
									.hull()
									.movey(-springRadius-4)
									.rotz(springboltRotation)
								
			def springPathDriveMountKW = moveDHValues(springMountKW.toZMax().movez(offsetSprings),dh)
			def springPathPassiveMountKW = moveDHValues(springMountKW.toZMin().movez(-springSupportLength/2-0.5),dh)
			
			def springPathDrive = moveDHValues(springPath.toZMax().movez(offsetSprings),dh)
			def springPathPassive = moveDHValues(springPath.toZMin().movez(-springSupportLength/2-0.5),dh)
			//end Spring path section				
								
			CSG MotorMountBracket = actuatorCircle.movez(-offsetOfLinks)
							.union(motorLink)
							.hull()
							.union(springPathDrive)
							.difference(vitamins)
							

			def FullBracket =CSG.unionAll([center,passiveSide,brace])
							//.difference(motorSidePlatekw.getBoundingBox())
							.difference(MotorMountBracket)
							.union(motorSidePlate)
							
							.difference([springPathDriveSideCutout])
							.difference(vitamins)
							.difference(motorToCut)
							.union(springPathPassive)
							.difference([springPathDriveMountKW,springPathPassiveMountKW])

								
			
			MotorMountBracket.setColor(javafx.scene.paint.Color.DARKCYAN)
			FullBracket.setColor(javafx.scene.paint.Color.YELLOW)
			MotorMountBracket.setManipulator(manipulator)
			FullBracket.setManipulator(manipulator)
			FullBracket.setName("MiddleLinkMainBracket")
			MotorMountBracket.setName("MiddleLinkActuatorBracket")
			allCad.addAll(FullBracket,MotorMountBracket)
		}

		if(linkIndex==0) {
			def z = dh.getD()-linkYDimention/2-movingPartClearence
			CSG hornKeepawy = moveDHValues(new Cylinder(hornDiameter/2,dh.getD()+1).toCSG()
					.movez(6),dh)
			
			def supportBeam= new RoundedCube(linkYDimention+linkThickness*2.0,40+linkThickness*2,z)
								.cornerRadius(cornerRad)
								.toCSG()
								
								.toZMax()
			def springSupport= new RoundedCube(springSupportLength,linkOneSupportWidth,z)
								.cornerRadius(cornerRad)
								.toCSG()
								.movex(-30)
								.toZMax()
			def	baseOfArm = Parabola.coneByHeight(baseCorRad, 25)
						.rotx(90)
						.toZMin()
						.movez(movingPartClearence)
						
			baseOfArm=baseOfArm
						.difference(
							baseOfArm
							.getBoundingBox()
							)
						.union(supportBeam.union(springSupport).movez(z+movingPartClearence))
						.transformed( TransformFactory.nrToCSG(locationOfBearing))
						.difference(vitamins)
						.difference(hornKeepawy)
			baseOfArm.setColor(javafx.scene.paint.Color.WHITE)
			baseOfArm.setManipulator(manipulator)
			baseOfArm.setName("BaseCone")
			baseOfArm.setManufacturing ({ mfg ->
				return mfg.rotx(-90).toZMin()				
			})
			allCad.add(baseOfArm)
		}
		//				CSG sparD = new Cube(gears.thickness,d.getDH_D(linkIndex),gears.thickness).toCSG()
		//						.toYMin()
		//						.toZMin()
		//				sparD.setManipulator(manipulator)
		//				allCad.add(sparD)
		d.addConnectionEventListener(new IDeviceConnectionEventListener (){

					/**
	 * Called on the event of a connection object disconnect.
	 *
	 * @param source the source
	 */
					public void onDisconnect(BowlerAbstractDevice source) {
							allCad.clear()
					}
					public void onConnect(BowlerAbstractDevice source) {}
				})
		for(CSG c:vitamins) {
			c.setManufacturing ({ mfg ->
			return null;
		})
		}
		allCad.addAll(vitamins)
		vitamins.clear()
		return allCad;
	}
	
	@Override
	public ArrayList<CSG> generateBody(MobileBase b ) {

		def vitaminLocations = new HashMap<TransformNR,ArrayList<String>>()
		ArrayList<CSG> allCad=new ArrayList<>();
		double baseGrid = grid;
		double baseBoltThickness=15;
		double baseCoreheight = 1;
		def insertMeasurments= Vitamins.getConfiguration(insert[0],
			insert[1])
		double xOffset = grid*7.5;
		double yOffset = -grid*0.5;
			
		

		for(DHParameterKinematics d:b.getAllDHChains()) {
			// Hardware to engineering units configuration
			LinkConfiguration conf = d.getLinkConfiguration(0);

			b.addConnectionEventListener(new IDeviceConnectionEventListener (){

						/**
		 * Called on the event of a connection object disconnect.
		 *
		 * @param source the source
		 */
						public void onDisconnect(BowlerAbstractDevice source) {
							//gears.clear()
							allCad.clear()
						}
						public void onConnect(BowlerAbstractDevice source) {}
					})

			CSG motorModel=   Vitamins.get(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())
			double zOffset = motorModel.getMaxZ()
			TransformNR locationOfMotorMount = d.getRobotToFiducialTransform().copy()
			TransformNR locationOfBearing = locationOfMotorMount.copy()
			//move for gearing
			
			// move the motor down to allign with the shaft
			if(locationOfBearing.getZ()>baseCoreheight)
				baseCoreheight=locationOfBearing.getZ()
			locationOfMotorMount.translateZ(-zOffset)
			TransformNR pinionRoot = locationOfMotorMount.copy().translateZ(topOfHornToBotomOfBaseLinkDistance+1)
			def extractionLocationOfMotor =locationOfMotorMount.copy().translateZ(-9)

			vitaminLocations.put(locationOfBearing.copy().translateZ(-1), [
				"ballBearing",
				thrustBearingSize
			])
			vitaminLocations.put(locationOfMotorMount.copy().translateZ(topOfHornToBotomOfBaseLinkDistance+1), [
				conf.getElectroMechanicalType(),
				conf.getElectroMechanicalSize()
			])
			vitaminLocations.put(extractionLocationOfMotor, [
				conf.getElectroMechanicalType(),
				conf.getElectroMechanicalSize()
			])
			// cut the hole in the base for the shaft
			vitaminLocations.put(pinionRoot, [
				conf.getShaftType(),
				conf.getShaftSize()
			])
		}
		
				
		def mountLoacions = [
			new TransformNR(baseGrid,0,0,new RotationNR(180,0,0)),//base
			new TransformNR(-baseGrid,baseGrid,0,new RotationNR(180,0,0)),//base
			new TransformNR(-baseGrid,-baseGrid,0,new RotationNR(180,0,0)),//base
			
		]
		mountLoacions.forEach{
			vitaminLocations.put(it.copy().translateZ(-10),
					["capScrew", boltsize])
			vitaminLocations.put(it.copy().translateZ(insertMeasurments.installLength),
					insert)

		}
	
				
		double totalMass = 0;
		TransformNR centerOfMassFromCentroid=new TransformNR();
		def vitamins=[]
		
		for(TransformNR tr: vitaminLocations.keySet()) {
			def vitaminType = vitaminLocations.get(tr)[0]
			def vitaminSize = vitaminLocations.get(tr)[1]

			HashMap<String, Object>  measurments = Vitamins.getConfiguration( vitaminType,vitaminSize)
			offset.setMM(offsetValue)
			CSG vitaminCad=   Vitamins.get(vitaminType,vitaminSize)
			Transform move = TransformFactory.nrToCSG(tr)
			CSG part = vitaminCad.transformed(move)
			part.setManipulator(b.getRootListener())
			vitamins.add(part)

			def massCentroidYValue = measurments.massCentroidY
			def massCentroidXValue = measurments.massCentroidX
			def massCentroidZValue = measurments.massCentroidZ
			def massKgValue = measurments.massKg
			//println "Base Vitamin "+vitaminType+" "+vitaminSize
			try {
				TransformNR COMCentroid = tr.times(
						new TransformNR(massCentroidXValue,massCentroidYValue,massCentroidZValue,new RotationNR())
						)
				totalMass+=massKgValue
			}catch(Exception ex) {
				BowlerStudio.printStackTrace(ex)
			}

			//do com calculation here for centerOfMassFromCentroid and totalMass
		}
		
		
		//Do additional CAD and add to the running CoM
		def thrustMeasurments= Vitamins.getConfiguration("ballBearing",
				thrustBearingSize)
		double baseCorRad = thrustMeasurments.outerDiameter/2+5
		CSG baseCore = new Cylinder(baseCorRad,baseCorRad,baseCoreheight,36).toCSG()
		CSG baseCoreshort = new Cylinder(baseCorRad,baseCorRad,baseCoreheight*3.0/4.0,36).toCSG()
		CSG mountLug = new Cylinder(15,15,baseBoltThickness,36).toCSG().toZMax()
		CSG mountCap = Parabola.coneByHeight(15, 8)
				.rotx(-90)
				.toZMax()
				.movez(-baseBoltThickness)
		CSG mountUnit= mountLug.union(mountCap)
		CSG allCore = baseCore.union(baseCore.movey(3).movex(3)).hull()
		CSG corBox=allCore.getBoundingBox().toXMin().toYMin();
		CSG calibrationCore = allCore
								.intersect(corBox)
								.intersect(corBox.rotz(-25))
								.rotz(-90)
		def coreParts=[baseCore]
		def boltHolePattern = []
		def boltHoleKeepawayPattern = []
		
		def bolt = new Cylinder(2.6,20).toCSG()
					.movez(-10)
		def boltkeepaway = new Cylinder(5,20).toCSG()
					.movez(-10)
		mountLoacions.forEach{
			
			def place =com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(it)
			boltHolePattern.add(bolt.transformed(place))
			boltHoleKeepawayPattern.add(boltkeepaway.transformed(place))
			coreParts.add(
					CSG.hullAll(mountLug
					.transformed(place)
					,baseCoreshort)
					)
			coreParts.add(mountCap
					.transformed(place)
					)
		}
		
		def locationOfCalibration = new TransformNR(0,-65.0,40, new RotationNR(-179.99,90,-60))
		DHParameterKinematics dev = b.getAllDHChains().get(0)
		//dev.setDesiredTaskSpaceTransform(locationOfCalibration, 0);
		def jointSpaceVect = dev.inverseKinematics(dev.inverseOffset(locationOfCalibration));
		def jointInts=[]
		def upperInts=[]
		def lowerInts=[]
		for(int i=0;i<jointSpaceVect.length;i++) {
			jointInts[i] = Math.round(jointSpaceVect[i]*100)
			upperInts[i]=Math.round(dev.getMaxEngineeringUnits(i)*100)
			lowerInts[i]=Math.round(dev.getMinEngineeringUnits(i)*100)
			
		}
		def poseInCal = dev.forwardOffset(dev.forwardKinematics(jointSpaceVect));
		println "\n\nCalibration Values "+jointInts+"\nUpper Lim= "+upperInts+"\nLower Lim"+lowerInts+"\nat pose: \n"+poseInCal+"\n\n"
				
		def calibrationFrame = TransformFactory.nrToCSG(locationOfCalibration)
								//.movex(centerlineToOuterSurfaceNegativeZ)
		def calibrationFramemountUnit=calibrationCore
										.rotx(180)
										//.toYMin()
										.toZMin()
										.movez(hornKeepawayLen)
										.transformed(calibrationFrame)
										.difference(new Cube(200, 200, 100).toCSG().toZMin().movez(-100))
										.difference(ScriptingEngine.gitScriptRun(
			"https://github.com/Halloween2020TheChild/GroguMechanicsCad.git", // git location of the library
			"wrist3.groovy" , // file to load
			// Parameters passed to the funcetion
			null
			).collect{it.transformed(calibrationFrame)}
			)
										//.toZMin()
										
		// assemble the base
		def calibrationTipKeepaway =new RoundedCylinder(linkYDimention/2,centerlineToOuterSurfacePositiveZ-centerlineToOuterSurfaceNegativeZ)
											.cornerRadius(cornerRad)
											.toCSG()
											.roty(-90)
									.transformed(calibrationFrame)
	
							
		def points = [	new Vector3d(10,0,0),
					new Vector3d(0, 0, 5),
					new Vector3d(0, -3, 0),
					new Vector3d(0, 3, 0)
		]
		CSG pointer = HullUtil.hull(points)
		
		def Base = CSG.unionAll(coreParts)
				.union(calibrationFramemountUnit)
				//.difference(vitamin_roundMotor_WPI_gb37y3530bracketOneKeepawayDistanceen)
				.difference(vitamins)
				//.difference(calibrationTipKeepaway)
				//.difference(cordCutter);
				
			

		//CSG boundingBase=Base.getBoundingBox()
		//Base = Base.intersect(boundingBase.toXMin().movex(-baseCorRad))	
		//Base = Base.intersect(boundingBase.toZMin())
		//Base = Base.union(pointer.movex(Base.getMaxX()-2))
		//				.union(pointer.rotz(90).movey(-baseCorRad+2))

		
		double extra = Math.abs(Base.getMinX())
		

		
		
	
		// hacky non vitamin hole solution
		
							
	
		allCad.addAll(Base)//cardboard,board,paper
		Base.addExportFormat("stl")
		Base.addExportFormat("svg")
		Base.setName("BaseMount")
		b.setMassKg(totalMass)
		b.setCenterOfMassFromCentroid(centerOfMassFromCentroid)
		
		allCad.addAll(vitamins)
		return allCad;
	}
};

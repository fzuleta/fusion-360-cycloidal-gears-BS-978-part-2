#Author-
#Description-
# heavy influenced by (pretty much copy-paste) from:
# https://www.csparks.com/watchmaking/CycloidalGears/index.jxl
# https://www.hessmer.org/gears/CycloidalGearBuilder.html

import adsk.core, adsk.fusion, adsk.cam, traceback, math

errorLimit=0.000001

class Form:
    def __init__(self, module, pinionCount, wheelCount):
        self.__compute__(module, pinionCount, wheelCount)

    def __format__(self, x):
        return int(x * 100000 + 0.5)/100000
    
    def __calculateAddendumFactor__(self, np, nw):
        b  = 0.0
        t0 = 1.0
        t1 = 0.0
        r2 = 2.0 * nw/np
    
        while (abs(t1 - t0) > errorLimit):	
            t0 = t1
            b = math.atan2(math.sin(t0), (1.0 + r2 - math.cos(t0)))
            t1 = math.pi/np + r2 * b	

        k = 1.0 + r2
        d = math.sqrt( 1.0 + k * k - 2.0 * k * math.cos(t1))
        result = 0.25 * np * (1.0 - k + d)

        return result
    def __getPinionAddendum__(self, toothCount, module):
        # returns (addendum, addendumRadius)
        # For details see the Profile - Leaves tables in http://www.csparks.com/watchmaking/CycloidalGears/index.jxl
        if (toothCount <= 7):
            # high ogival
            return (0.855 * module, 1.050 * module) 
        elif toothCount == 8 or toothCount == 9:
            # medium ogival
            return (0.670 * module, 0.7 * module) 

        elif (toothCount == 10):
            # round top for small tooth 
            return (0.525 * module, 0.525 * module)
        else:
            # 11+ teeth, round top for wider tooth
            return (0.625 * module, 0.625 * module)
	
    def __getWheelDedendum__(self, customSlop, pinionAddendum, module, wheelPitchDiameter, wheelAddendum):
        d = module * math.pi / 2
        if (customSlop >= 0):
            d = pinionAddendum + customSlop
        innerRadius = wheelPitchDiameter / 2 - d
        outerRadius = wheelPitchDiameter / 2 + wheelAddendum
        return (d, innerRadius, outerRadius)
	
    def __getPinionDedendum__(self, customSlop, pinionAddendum, module, pinionPitchDiameter, wheelAddendum, addendumFactor, toothCount):
        d = module * (addendumFactor + 0.4)
        if (customSlop >= 0):
            d = wheelAddendum + customSlop
        innerRadius = pinionPitchDiameter / 2 - d
        outerRadius = pinionPitchDiameter / 2 + pinionAddendum
        offset = 180 + 180 / toothCount
        return (d, innerRadius, outerRadius, offset)
        
    def __compute__(self, m, np, nw):
        af = self.__calculateAddendumFactor__(np, nw)
        pf = 0.95 * af
        gr = nw / np

        cp = m * math.pi
        dd = m * math.pi/2

        dw = m * nw
        dp = m * np
        ad = m * 0.95 * af
        ar = m * 1.40 * af
        
        self.gearRatio           = self.__format__(gr)
        self.circularPitch       = self.__format__(cp)
        self.dedendum            = self.__format__(dd)
        self.wheelPitchDiameter  = self.__format__(dw)
        self.pinionPitchDiameter = self.__format__(dp)
        self.wheelPinionDistance = (dw + dp) / 2.0
        self.addendumFactor      = self.__format__(pf)
        
        self.wheelToothCount     = nw
        self.wheelAddendum       = self.__format__(ad)
        self.wheelAddendumRadius = self.__format__(ar)
        self.wheelHalfToothAngle = self.__format__(math.pi / nw / 2)
        
        pinionFactor = 1.05 if np <= 10 else 1.25
        self.pinionHalfToothAngle = self.__format__(pinionFactor * m / dp)
        padd = self.__getPinionAddendum__(np, m)
        self.pinionAddendum = self.__format__(padd[0])
        self.pinionAddendumRadius = self.__format__(padd[1])
        
        wheelDedendum = self.__getWheelDedendum__(-1, self.pinionAddendum, m, self.wheelPitchDiameter, self.wheelAddendum)
        self.wheelDedendum = wheelDedendum[0]
        self.wheelInnerRadius = wheelDedendum[1]
        self.wheelOuterRadius = wheelDedendum[2]
        
        self.pinionToothCount  = np
        pinionDedendum = self.__getPinionDedendum__(-1, self.pinionAddendum, m, self.pinionPitchDiameter, self.wheelAddendum, self.addendumFactor, np)
        self.pinionDedendum = pinionDedendum[0]
        self.pinionInnerRadius = pinionDedendum[1]
        self.pinionOuterRadius = pinionDedendum[2]
        self.pinionAngle = pinionDedendum[3]
        self.pinionCenter = [(dw + dp) / 2, 0]

# HELPER FUNCTIONS

def calculate_arc_center(start_point, end_point, radius):
    """Calculate the center point of an arc given the start point, end point, and radius."""
    # Calculate the midpoint of the line segment between start and end points
    mid_x = (start_point.x + end_point.x) / 2
    mid_y = (start_point.y + end_point.y) / 2
    mid_point = adsk.core.Point3D.create(mid_x, mid_y, 0)

    # Calculate the distance between the start and end points
    dist = math.sqrt((end_point.x - start_point.x) ** 2 + (end_point.y - start_point.y) ** 2)

    # Calculate the distance from the midpoint to the center of the arc
    half_chord = dist / 2
    if radius < half_chord:
        raise ValueError("Radius is too small to form an arc with the given start and end points")

    center_offset = math.sqrt(radius**2 - half_chord**2)

    # Determine the direction of the arc (above or below the chord)
    # Here, we place the center above the line segment (for a clockwise arc)
    dx = end_point.x - start_point.x
    dy = end_point.y - start_point.y
    perp_x = -dy / dist  # Perpendicular direction to the line segment
    perp_y = dx / dist

    # Calculate the center point of the arc
    center_x = mid_x + center_offset * perp_x
    center_y = mid_y + center_offset * perp_y
    return adsk.core.Point3D.create(center_x, center_y, 0)

def createWheel(app: adsk.core.Application, ui: adsk.core.UserInterface, pitchDiameter: float, addendum: float, addendumRadius: float, dedendum: float, toothCount: int, halfToothAngle, initPoint):
    design = app.activeProduct
    rootComp: adsk.fusion.Component = design.rootComponent
    sketches = rootComp.sketches
    extrudes = rootComp.features.extrudeFeatures
    xyPlane = rootComp.xYConstructionPlane
    sketch = sketches.add(xyPlane)
    
    circles = sketch.sketchCurves.sketchCircles
    dedendumCircle = circles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), pitchDiameter / 2.0 - dedendum)
    
    toothEntities = createProtoTooth(sketch, pitchDiameter, addendum, halfToothAngle)
    
    normal = sketch.xDirection.crossProduct(sketch.yDirection)
    normal.transformBy(sketch.transform)
    origin = sketch.origin
    origin.transformBy(sketch.transform)
    rotationMatrix = adsk.core.Matrix3D.create()
    step = 2 * math.pi / toothCount
            
    for i in range(1, int(toothCount)):
        rotationMatrix.setToRotation(step * i, normal, origin)
        sketch.copy(toothEntities, rotationMatrix)
    
    profs = adsk.core.ObjectCollection.create()
    for prof in sketch.profiles:
        profs.add(prof)
        
    
    # Perform an extrusion using the selected profile
    extrudeInput = extrudes.createInput(profs, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extrudeInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(0.55))

    # Create the extrusion
    extrudeFeature = extrudes.add(extrudeInput)
    
        
def createProtoTooth(sketch, pitchDiameter: float, addendum: float, halfToothAngle):
    centerRayAngle = 0
    radius = pitchDiameter / 2.0 + addendum
    center = [0,0]
    apex = [
        center[0] + math.cos(centerRayAngle) * radius,
        center[1] + math.sin(centerRayAngle) * radius
    ]
    
    leftFlankAngle = centerRayAngle - halfToothAngle
    rightFlankAngle = centerRayAngle + halfToothAngle
    
    pitchRadius = pitchDiameter / 2.0
    pitchCircleIntersectLeft = [
        center[0] + math.cos(leftFlankAngle) * pitchRadius,
        center[1] + math.sin(leftFlankAngle) * pitchRadius
    ]
    pitchCircleIntersectRight = [
        center[0] + math.cos(rightFlankAngle) * pitchRadius,
        center[1] + math.sin(rightFlankAngle) * pitchRadius
    ]
    
    lines = sketch.sketchCurves.sketchLines
    arcs = sketch.sketchCurves.sketchArcs
    
    c = adsk.core.Point3D.create(center[0], center[1], 0)
    line1 = lines.addByTwoPoints(c, adsk.core.Point3D.create(pitchCircleIntersectLeft[0], pitchCircleIntersectLeft[1], 0))
    
    sp0 = adsk.core.Point3D.create(pitchCircleIntersectLeft[0], pitchCircleIntersectLeft[1], 0)
    sp1 = adsk.core.Point3D.create(apex[0], apex[1], 0)
    arc1 = arcs.addByCenterStartEnd(calculate_arc_center(sp0, sp1, radius), sp0, sp1)
    
    sp2 = adsk.core.Point3D.create(pitchCircleIntersectRight[0], pitchCircleIntersectRight[1], 0) 
    arc2 = arcs.addByCenterStartEnd(calculate_arc_center(sp1, sp2, radius), sp1, sp2)
    
    line2 = lines.addByTwoPoints(sp2, c)
    
    inputEntites = adsk.core.ObjectCollection.create()
    inputEntites.add(line1)
    inputEntites.add(arc1)
    inputEntites.add(arc2)
    inputEntites.add(line2)
    return inputEntites

def run(context):
    
    app = adsk.core.Application.get()
    ui  = app.userInterface
    
    try:
        f = Form(0.13, 17, 112)
        
        # Create Wheel
        createWheel(
            app,
            ui,
            f.wheelPitchDiameter, 
            f.wheelAddendum, 
            f.wheelAddendumRadius,
            f.wheelDedendum, 
            f.wheelToothCount,
            f.wheelHalfToothAngle,
            adsk.core.Vector3D.create(0, 0, 0)
            )
        
        # Create Pinion
        createWheel(
            app,
            ui,
            f.pinionPitchDiameter, 
            f.pinionAddendum, 
            f.pinionAddendumRadius,
            f.pinionDedendum, 
            f.pinionToothCount,
            f.pinionHalfToothAngle,
            adsk.core.Vector3D.create(f.pinionCenter[0], f.pinionCenter[1], 0)
            )
        
        # ui.messageBox(f"""
        #     gearRatio: {str(f.gearRatio)} 
        #     """)
        
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

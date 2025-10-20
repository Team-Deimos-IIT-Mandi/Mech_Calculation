friction = float(input("the fricition b/w pades and rotor: ")) 
RotarR=float(input("radius of disc: ")) #mm
outerR = float(input("outer radius of the pad from the center: ")) #mm
innerR = float(input("inner radius of the pad from the center: ")) #mm
normalForce = None # force given by the pads to the rotor #N (equal to the cable tension as mechanical advantage is 1)
effectiveRadius = (2 * ((outerR**3) - (innerR**3))) / (3 * ((outerR**2) - (innerR**2))) # mm
effectiveRadius = effectiveRadius / 1000 #m
mechanicalAdvantage = float(input("enter the MA: ")) # 1,if no lever used
torqueB = float(input("eneter the torque need to be balnaced: ")) #Nm

normalForce = torqueB / (friction * effectiveRadius * mechanicalAdvantage) #N
print(normalForce)
#from the given paper we know 15mm is the cable to be pulled for max tension tranfer
radiusM = float(input("enter the shaft radius of the motor/hub : ")) #mm
radiusM = radiusM / 1000 #m
torqueM = normalForce*radiusM #Nm
print(torqueM * 10.197162129779) #kgcm
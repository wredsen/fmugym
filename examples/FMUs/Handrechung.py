import math
#test

#Listen für FR Worst Case (min und max), Zuordnung erfolgt manuell
FR1_min = []
FR1_max =[]
FR1_nom = 0.5

# R1 resultiert aus AD des Rotors, einfluss auf FR ist direkt linear (1D-Tol-Kette), Wert ist gleich Toleranzfeldbreite
R1 = 0.1
FR_R1_min = FR1_nom - R1/2
FR_R1_max = FR1_nom + R1/2
FR1_min.append(FR_R1_min-FR1_nom)
FR1_max.append(FR_R1_max-FR1_nom)

# S2 kommt vom ID des Stators, ansonsten identisch zu R1
S2 = 0.1
FR_S2_min = FR1_nom - S2/2
FR_S2_max = FR1_nom + S2/2
FR1_min.append(FR_S2_min-FR1_nom)
FR1_max.append(FR_S2_max-FR1_nom)

# Kinematik der nachfolgenden Betrachtungen ergibt sich aus RBs/Joints in CETOL
# C4 ist verschobene Position der Kugelkalotte im Case nach oben/unten
# Kinematische reaktion: R verkippt um Kontaktpunkt B1, Drheung am Kontaktpunkt R3-B2 auch möglich, verschiebung findet in C im Mittelpunkt B2 statt
# Verbindung von C4 zu S besteht über S3-C2 (zulässig: ty,r; unterdrück: tx) und S4-C1 (zulässig: tx, ty, r) 
# aktuell ist in CETOL ty über S5-C3 definiert und verschiebt damit wohl den Drehpunkt, da dieser Joint bei 2D entfällt, am besten umdefinieren
# --------- Denk-cut --------
# C sitzt centered aus B2 -> muss also um B2 drehen, in x-Richtung ist der Kontaktpunkt S3-C2 blockiert -> drehung um B2 so weit, dass sich diese x-Koordinate nicht ändert
C4 = 0.1

# Koordinaten von FR1 relativ zu drehpunkt
FR1_kipp_x = 13.96
FR1_kipp_y = 7.25
# Geometrie und Hilfsgrößen
a=4
t_trans= -C4   # = - C4/2, VZ von t ist umgekehrt zu globalen Koordinaten, Toleranzfeld symmetrisch, damit die Werte stimmen muss hier C4 rein, warum nciht C4/2?
l=30.5
A = a/t_trans
B=a/l-1
C = -B**2 + A**2

# Berechnung C4 Kippen
X1 = A/C + math.sqrt(A**2/C**2 - (1-B**2)/C)    # für t positiv (negative y Richtung in globalkoordinaten (eventuell drehen))
X2 = A/C - math.sqrt(A**2/C**2 - (1-B**2)/C)    # für t negativ (pos y in glob Koord)
alpha1 = math.atan(X1)
alpha2 = math.atan(X2)
#print(X1, X2)
#alpha1 = math.degrees(alpha1)
#alpha2 = math.degrees(alpha2)
#print(alpha1, alpha2)
FR1_C4_min = FR1_kipp_y*(1-math.cos(alpha1)) + FR1_kipp_x*math.sin(alpha1)   # min Abstand ergibt sich bei Kippen nach oben (alpha2)
FR1_C4_max = -FR1_C4_min  # max Abstand ergibt sich bei Kippen anch oben (alpha1). Da verhalten symmetrisch ist, kann einfach negativer Wert übernommen werden, statt neu zu rechnen
FR1_min.append(FR1_C4_min)  
FR1_max.append(FR1_C4_max)

# Berechnungen fuer Bewegung der Cap benötigen geometrische Parameter sowie je zwei Betrachtungen für r und y bei S4
# Option: S4 in CETOL 0 setzen, damits einfacher wird? Dann ignorieren wir sehr viel, aber immerhin bleibt es im paper nachvollziehbar
# rein logisch und auch nach CETOL Ergebnis ist S4 Vershciebung in y identisch zu C4 in y -> selbe Ergebnisse
S4 = 0.2
S4_ty = S4/2  # Für lineare Verschiebung ist die Toleranz genau die halbe Toleranzfeldbreite ToDo: hier stimmat was nicht, weil Eregbnis in CETOL identisch zu C4, obwohl anderer Toleranzwert
# aktuell Annahme beide sind gleich und Wert wird kopiert, ToDo: schreibe Funktion für Verkipprechnung oder kopiere Formel
FR1_S4_ty_min = FR1_C4_min    #ToDo: Funktionsaufruf mit Toleranswert von S4_ty bzw. Formel nochmal hinklatschen, damit sich C4 und S4 unabhängige inputs sind
FR1_S4_ty_max = -FR1_S4_ty_min  
FR1_min.append(FR1_S4_ty_min)  
FR1_max.append(FR1_S4_ty_max)
# S4 rot sehr kleine Einflüsse -> Berechnung herleiten
# Annahme: kineamtik ist die selbe wie bisher, vorgegeben wird aber der Kippwinkel der Cap C
# Rotation von C entspricht Winkel beta in der Herleitung der Kinematik -> Beziehung beta zu alpha sollte hier die Lösung geben
#S4_r = 0.00177 #0.0177 # für Rotation leitet sich aus der Toleranzfeldbreite des Statorzylinders aus der Positionstoleranz der Zylinderfläche ab: maximale Verkippung in Toleranzzone =
S4_r = math.atan(S4/113) # empirisch aus Ergebnis von CETOL, geometrische Zusammenhänge beim CETOL Termin genau abklären, woher kommt die Länge 113 mm?
# Kippen von C entspricht Winkel beta aus Herleitung C4, alpha hier dann über Gleichungen (1) und (a) aus Herleitung C4 lösbar:
beta = S4_r 
alpha = math.acos((a*math.cos(beta)+l-a)/l)
FR1_S4_r_max = FR1_kipp_y*(1-math.cos(alpha)) + FR1_kipp_x*math.sin(alpha) #Vorzeichen der y Koordinate wieder gedreht
FR1_S4_r_min = -FR1_S4_r_max
FR1_min.append(FR1_S4_r_min)  
FR1_max.append(FR1_S4_r_max)


print(FR1_min,FR1_max)
print("Summe:",sum(FR1_min),sum(FR1_max))
print("Grenzwerte:",FR1_nom+sum(FR1_min),FR1_nom+sum(FR1_max))


#### FR2 ###
# !!! Für FR2 ist Poition von N relevant -> muss fest verknüpft und Maße bekannt sein !!! Z-Bewegung in R4-N1 in CETOL sperren, wenn unmöglich R und C zu einem Teil kombinieren!
# Bei Anpassung auf 2D wird hier vermutlich einiges satrk anders sein, was tun wenn CETOL kein vernünftiges 2D machen will? Wie Bewegung in Weben verhindern? 
# Bauteil kommtlett Rotationssymmetrich machen?
# Alternative: unseren 2D Ansatz mit CETOL 3D vergleichen (erklärt dann auch gleich ne Menge Abweichungen ^^)

#Listen für FR Worst Case (min und max), Zuordnung erfolgt manuell
FR2_min = []
FR2_max =[]
FR2_nom = 0.15

# Koordinaten von FR2 relativ zu drehpunkt für Kippen des Rotors und Wackeln der Mutter (aktuell ist x noch nicht fest parametriert, muss in CETOL fixiert werden)
# aktuelle Werte nur grob aus SW übernommen
FR2_kipp_x = 31.2
FR2_kipp_y = 5
FR2_wackel_x = 17
FR2_wackel_y = 5

# weitestgehend von oben übernehmen, eigene lineare abweichungen und anderes FR_nom + zusätzlich lineare Werte von R4, verkippen C4 und S4 hat anderen FR Vektor beim drehen um alpha, sonst gleich

# C5
C5 = 0.1
FR2_min.append(-C5/2)
FR2_max.append(C5/2)

# R4_y
R4 = 0.1
FR2_min.append(-R4/2)
FR2_max.append(R4/2)

# neu R4_rotation: reine Rotation von N auf R aufgrund von Lauftoleranz R
# zu bestimmende Parameter: Vektor von Drehpunkt zu FR (fix), Toleranzzone (in CETOL nachlesen oder irgendwie selbst aus Lauftoleranzbereich ableiten vergleiche S4)
# R4_r
R4_r = math.atan(R4/113)    # geometrische Parameter hier später anders als bei S4
FR2_R4_r_min = FR2_wackel_y*(1-math.cos(R4_r)) + FR2_wackel_x*math.sin(R4_r)   
FR2_R4_r_max = FR2_R4_r_min 
FR2_min.append(FR2_R4_r_min)  
FR2_max.append(FR2_R4_r_max)

# N2
N2 = 0.1
FR2_min.append(-N2/2)
FR2_max.append(N2/2)

# C4 (s.o. mit angepassten Koordinaten für FR2)
# C4 Input oben schon definiert

# Berechnung X1/2 und alpha 1/2 ist identisch wie oben und nutzt auch selben input. -> in Modelica an die selbe Stelle packen? wäre übersichtlicher
FR2_C4_min = FR2_kipp_y*(1-math.cos(alpha1)) + FR2_kipp_x*math.sin(alpha1)   
FR2_C4_max = -FR2_C4_min  
FR2_min.append(FR2_C4_min)  
FR2_max.append(FR2_C4_max)

# S4 (s.o. mit angepassten Koordinaten für FR2)
#S4_ty
FR2_S4_ty_min = FR2_C4_min    
FR2_S4_ty_max = -FR2_S4_ty_min  
FR2_min.append(FR2_S4_ty_min)  
FR2_max.append(FR2_S4_ty_max)
#S4_r
FR2_S4_r_max = FR2_kipp_y*(1-math.cos(alpha)) + FR2_kipp_x*math.sin(alpha) 
FR2_S4_r_min = -FR2_S4_r_max
FR2_min.append(FR2_S4_r_min)  
FR2_max.append(FR2_S4_r_max)


print(FR2_min,FR2_max)
print("Summe:",sum(FR2_min),sum(FR2_max))
print("Grenzwerte:",FR2_nom+sum(FR2_min),FR2_nom+sum(FR2_max))

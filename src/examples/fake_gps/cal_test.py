import math
M_PI = 3.14159265358979323846
a = 6378.1370 # Earth radius km
b = 6356.7523 # Earth radius km
Ly = 2.0*M_PI*math.sqrt(0.5*(math.pow(a,2)+math.pow(b,2)))
print("Ly", Ly)
theta = 0 *M_PI /180.
print("theta", theta)
km2lat = 360./Ly
lat2km = Ly/360.
print("km2lat", km2lat)
print("lat2km", lat2km)
Rytheta = math.sqrt(pow(a*b,2)/(math.pow(b,2)+math.pow(a*math.tan(theta),2)))
km2lon = 180./(M_PI*Rytheta)
lon2km = (M_PI*Rytheta)/180
print("lon2km", lon2km)
print("2piRytheta", 2*M_PI*Rytheta)
b = 6378.1370 # Earth radius km
a = 6356.7523 # Earth radius km
Rytheta = math.sqrt(pow(a*b,2)/(math.pow(a,2)+math.pow(b*math.tan(theta),2)))
print("2piRytheta", 2*M_PI*Rytheta)
km2lon = 180./(M_PI*Rytheta)
lon2km = (M_PI*Rytheta)/180
print("lon2km", lon2km)

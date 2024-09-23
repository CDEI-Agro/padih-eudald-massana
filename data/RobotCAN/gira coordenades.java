import java.lang.Math;

public class Main {
    public static void main(String[] args) {
        double[] result = calculateNewCoordinates(40.7128, -74.0060, 45, 10000);
        System.out.println("New Latitude: " + result[0] + ", New Longitude: " + result[1]);
    }

    public static double[] calculateNewCoordinates(double lat, double lon, double angle, double distance) {
        // Convertimos los grados a radianes
        lat = Math.toRadians(lat);
        lon = Math.toRadians(lon);
        angle = Math.toRadians(angle);

        // Radio de la Tierra en metros
        double earthRadius = 6371000;

        // Calculamos las nuevas coordenadas
        double newLat = Math.asin(Math.sin(lat) * Math.cos(distance / earthRadius) + Math.cos(lat) * Math.sin(distance / earthRadius) * Math.cos(angle));
        double newLon = lon + Math.atan2(Math.sin(angle) * Math.sin(distance / earthRadius) * Math.cos(lat), Math.cos(distance / earthRadius) - Math.sin(lat) * Math.sin(newLat));

        // Convertimos las coordenadas de vuelta a grados
        newLat = Math.toDegrees(newLat);
        newLon = Math.toDegrees(newLon);

        return new double[]{newLat, newLon};
    }
}

/*
Esta función toma como entrada la latitud y longitud del punto de origen,
el ángulo en grados y la latitud y longitud del punto de destino. Devuelve las nuevas coordenadas en grados.
*/

import java.lang.Math;

public class Main {
    public static void main(String[] args) {
        double[] result = calculateNewCoordinates(40.7128, -74.0060, 45, 40.7129, -74.0061);
        System.out.println("New Latitude: " + result[0] + ", New Longitude: " + result[1]);
    }

    public static double[] calculateNewCoordinates(double lat1, double lon1, double angle, double lat2, double lon2) {
        // Convertimos los grados a radianes
        lat1 = Math.toRadians(lat1);
        lon1 = Math.toRadians(lon1);
        lat2 = Math.toRadians(lat2);
        lon2 = Math.toRadians(lon2);
        angle = Math.toRadians(angle);

        // Radio de la Tierra en metros
        double earthRadius = 6371000;

        // Calculamos la distancia entre los dos puntos
        double dLat = lat2 - lat1;
        double dLon = lon2 - lon1;
        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2) + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        double distance = earthRadius * c;

        // Calculamos las nuevas coordenadas
        double newLat = Math.asin(Math.sin(lat1) * Math.cos(distance / earthRadius) + Math.cos(lat1) * Math.sin(distance / earthRadius) * Math.cos(angle));
        double newLon = lon1 + Math.atan2(Math.sin(angle) * Math.sin(distance / earthRadius) * Math.cos(lat1), Math.cos(distance / earthRadius) - Math.sin(lat1) * Math.sin(newLat));

        // Convertimos las coordenadas de vuelta a grados
        newLat = Math.toDegrees(newLat);
        newLon = Math.toDegrees(newLon);

        return new double[]{newLat, newLon};
    }
}
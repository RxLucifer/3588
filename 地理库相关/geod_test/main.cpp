

#include <GeographicLib/Geodesic.hpp> 
//#include <algorithm>  
#define _USE_MATH_DEFINES
#include <cmath>  
#include <iostream>

using namespace std;

#define M_PI 3.14159265358979323846

double calculate_delta_angle(double angle1, double angle2)
{
	while (angle1 > 360) { angle1 -= 360; }
	while (angle1 < 0) { angle1 += 360; }
	while (angle2 > 360) { angle2 -= 360; }
	while (angle2 < 0) { angle2 += 360; }
	double delta = angle1 - angle2;
	if (delta > 180)
	{
		delta = delta - 360;
	}
	else if (delta < -180)
	{
		delta = 360 + delta;
	}
	return delta;
}
//3.����Ǽ���
double Azimuth_withGeo(double lat1, double lon1, double lat2, double lon2)
{
	double s12(0), azi1(0), azi2(0);
	const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
	geod.Inverse(lat1, lon1, lat2, lon2, s12, azi1, azi2);
	return azi1;
}

double calc_Lf(double v)
{
	double k = 2.552, b = 0;
	double Ld = k * v + b;
	if (Ld < 2.90) {
		Ld = 2.90; //���ǽ��ջ��ͺ���֮��ľ��룬��2.85m����0.5m ��Ҫ�Ķ�
	}

	//Ld = 2.85; //5kph
	//Ld = 5.7; //10kph 15kph
	return Ld;
}

//�ںϺ�ĵ��ô���
/*
��������
vehicle_speed: ��ǰ�����ٶȣ���λ��km/h����
latitude: ��ǰ������γ�ȡ�
longitude: ��ǰ�����ľ��ȡ�
heading_angle: ��ǰ�����ĺ���ǣ���λ���ȣ���
map_latitude_v: ��ͼ�����е��γ���б���
map_longitude_v: ��ͼ�����е�ľ����б���
map_heading_angle_v: ��ͼ�ϸ���ĺ�����б���
wheelbase: ��������ࣨ��λ���ף���
�������
steer_value:Ѱ�����������ķ�����ת�ǡ�


*/
double control(double vehicle_speed, double latitude, double longitude, double heading_angle,
	double map_latitude_v, double map_longitude_v, double map_heading_angle_v,
	double wheelbase)
{
	double steer_value = 999;
	// ����Ԥ����� Ld
	double Ld = calc_Lf(vehicle_speed / 3.6);

	// ���ݳ��ٵ���Ԥ�����
	if (vehicle_speed < 0) {
		Ld = 2.90;  // �ٶ�С��0ʱ�趨��СԤ�����
	}
	else if (vehicle_speed >= 0 && vehicle_speed < 30) {
		// Ԥ����벻�õ���
	}
	else if (vehicle_speed >= 30 && vehicle_speed < 45) {
		Ld -= 7;  // ����Ԥ�����
	}
	else if (vehicle_speed >= 45 && vehicle_speed < 60) {
		Ld -= 10.5;  // ����Ԥ�����
	}

	// ���ҵ�ͼ����
	// int ind = calc_target_index(latitude, longitude, Ld, map_latitude_v, map_longitude_v);

	// double map_latitude = map_latitude_v[ind < map_latitude_v.size() ? ind : map_latitude_v.size() - 1];
	// double map_longitude = map_longitude_v[ind < map_longitude_v.size() ? ind : map_longitude_v.size() - 1];
	// double map_heading_angle = map_heading_angle_v[ind < map_heading_angle_v.size() ? ind : map_heading_angle_v.size() - 1];

	double map_latitude = map_latitude_v;
	double map_longitude = map_longitude_v;

	// ���㴿���ٿ��Ƶ�ת��
	double alpha = calculate_delta_angle(Azimuth_withGeo(latitude, longitude, map_latitude, map_longitude), heading_angle);

	// ���ݳ��ٵ�������Ч��
	double delta = -1.0 * atan2(2.0 * wheelbase * sin(alpha / 180.0 * M_PI) / Ld, 1.0) * 180.0 / M_PI;

	// ����ת�Ƿ�Χ
	double bound = 25.59402924599883;  // ǰ��ת���� ��Ҫ����ʵ�ʵ���
	if (delta > bound) {
		delta = bound;
	}
	if (delta < -bound) {
		delta = -bound;
	}

	//��ǰ���������̱���ϵ�� ��ʵ�ʷ����̽Ƕ�=��������������ƫ�� * ����ϵ���� ����ϵ����Ϊ�����̴���ٺ���ƫת1����λ��
	float Cal = 18.363658003300756;  //��Ҫ����
	// ���ؼ���ķ�����ת�ǣ�steer_value��
	steer_value = delta * Cal;

	return steer_value;
}

constexpr double toRadians(double degree) {
	return degree * M_PI / 180.0;
}

// ������ת��Ϊ�Ƕ�
constexpr double toDegrees(double radian) {
	return radian * 180.0 / M_PI;
}

// ��������GPS������������ļн�
double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
	// ת��Ϊ����
	lat1 = toRadians(lat1);
	lon1 = toRadians(lon1);
	lat2 = toRadians(lat2);
	lon2 = toRadians(lon2);

	// ���㾭�Ȳ�
	double dLon = lon2 - lon1;

	// ʹ�ù�ʽ���㺽���
	double y = sin(dLon) * cos(lat2);
	double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
	double bearing = atan2(y, x);

	// ת��Ϊ 0�� �� 360�� ��Χ
	return fmod((toDegrees(bearing) + 360), 360);
}

// ʹ�� Haversine ��ʽ�������
double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
	const double R = 6371.0; // ����ƽ���뾶����λ������

	// ת��Ϊ����
	lat1 = toRadians(lat1);
	lon1 = toRadians(lon1);
	lat2 = toRadians(lat2);
	lon2 = toRadians(lon2);

	// ���㾭γ�Ȳ�
	double dLat = lat2 - lat1;
	double dLon = lon2 - lon1;

	// Haversine ��ʽ
	double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
		std::cos(lat1) * std::cos(lat2) *
		std::sin(dLon / 2) * std::sin(dLon / 2);
	double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

	// ����
	return R * c;
}

double distance(double lat1, double lon1, double lat2, double lon2) {
	double dLat = (lat2 - lat1) * M_PI / 180.0;
	double dLon = (lon2 - lon1) * M_PI / 180.0;

	lat1 = lat1 * M_PI / 180.0;
	lat2 = lat2 * M_PI / 180.0;

	double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	return 6378.140 * c * 1000;
}


int main()
{
	double latitude, longitude, heading_angle, map_latitude, map_longitude;
	double s12(0), azi1(0), azi2(0);

	latitude = 39.06083552;
	longitude = 117.05746754;
	map_latitude = 39.06083279;
	map_longitude = 117.05748728;
	const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
	geod.Inverse(latitude, longitude, map_latitude, map_longitude, s12, azi1, azi2);

	printf("\ns12: %f\nazi1: %f\nazi2: %f\n", s12, azi1, azi2);

	double angle = calculateBearing(latitude, longitude, map_latitude, map_longitude);

	//angle -= 360;

	double dis = distance(latitude, longitude, map_latitude, map_longitude);

	printf("\nangle: %f\n", angle);
}

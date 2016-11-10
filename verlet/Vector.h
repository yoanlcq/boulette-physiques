#ifndef __VECTOR_H__
#define __VECTOR_H__

#include <math.h>

struct Vec2 { //Simple vector class
	float X;
	float Y;

	Vec2( float pX = 0.0f, float pY = 0.0f ) : X( pX ), Y( pY ) { }

	~Vec2() { }

	Vec2 operator*( float Scalar ) { return Vec2( X*Scalar, Y*Scalar ); }
	Vec2 operator/( float Scalar ) { return Vec2( X/Scalar, Y/Scalar ); }
	Vec2 operator+( Vec2 B ) { return Vec2( X + B.X, Y + B.Y ); }
	Vec2 operator-( Vec2 B ) { return Vec2( X - B.X, Y - B.Y ); }
	Vec2 operator-() { return Vec2( -X, -Y ); }
	float operator*( Vec2 B ) { return X*B.X + Y*B.Y; }
	void operator=( Vec2 B ) { X = B.X; Y = B.Y; }
	void operator+=( Vec2 B ) { X += B.X; Y += B.Y; }
	void operator-=( Vec2 B ) { X -= B.X; Y -= B.Y; }
	void operator*=( float Scalar ) { X *= Scalar; Y *= Scalar; }
	void operator/=( float Scalar ) { X /= Scalar; Y /= Scalar; }
	bool operator==( Vec2 B ) { return X == B.X && Y == B.Y; }
	bool operator!=( Vec2 B ) { return X != B.X || Y != B.Y; }

	float Length() { return sqrtf( X*X + Y*Y ); }
	float LengthSq() { return X*X + Y*Y; }

	void Normalize() {
		float Len = 1.0/sqrtf( X*X + Y*Y );

		X *= Len;
		Y *= Len;
	}
};

#endif

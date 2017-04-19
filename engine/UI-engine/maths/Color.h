#ifndef COLOR_H
#define COLOR_H


namespace utility_engine
{
	// Structure Color
	// This structure represents a RGBA color.
	struct Color
	{

		public:

			// -------------------- Attributes -------------------- //

			// RGBA color components
			float r, g, b, a;

			// -------------------- Methods -------------------- //

			// Constructor
			Color() : r(1), g(1), b(1), a(1) {}

			// Constructor
			Color(float r, float g, float b, float a) : r(r), g(g), b(b), a(a) {}

			// Constructor
			Color(const Color& color) : r(color.r), g(color.g), b(color.b), a(color.a) {}

			// Destructor
			~Color() {}

			// Return the black color
			static Color black() { return Color(0.0f, 0.0f, 0.0f, 1.0f);}

			// Return the white color
			static Color white() { return Color(1.0f, 1.0f, 1.0f, 1.0f);}

			// = operator
			Color& operator=(const Color& color) {
				if (&color != this) {
					r = color.r;
					g = color.g;
					b = color.b;
					a = color.a;
				}
				return *this;
			}
	};

}
#endif

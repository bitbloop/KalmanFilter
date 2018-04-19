#pragma once


namespace renderer
{
	namespace quad
	{
		unsigned int create(const unsigned int program_id, const bool make_cube = false );
		void kill(const unsigned int quad_id);
		void render(const unsigned int quad_id);
	};
};

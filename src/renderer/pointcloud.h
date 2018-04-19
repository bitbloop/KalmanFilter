#pragma once


namespace renderer
{
	namespace pointcloud
	{
		unsigned int create(const unsigned int program_id, const unsigned int point_num, const float* data_xyz, const float* data_rgb = nullptr);
		void kill(const unsigned int pointcloud_id);
		void render(const unsigned int pointcloud_id, const bool render_as_line_strip = false);
	};
};

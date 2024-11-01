#ifndef RVIZ_CUSTOM_PLUGINS__POLYGON_ARRAY_DISPLAY_HPP_
#define RVIZ_CUSTOM_PLUGINS__POLYGON_ARRAY_DISPLAY_HPP_

#include <Ogre.h>
#include <OgreManualObject.h>

#include <queue>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "robot_interfaces/msg/polygon_array_stamped.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"

namespace rviz_custom_plugins
{
	enum class DisplayMode : int
	{
		Triangle = 1,
		Line = 2
	};

	template <DisplayMode mode>
	std::string get_display_mode_name();

	class PolygonArrayDisplay : public rviz_common::MessageFilterDisplay<robot_interfaces::msg::PolygonArrayStamped>
	{
			Q_OBJECT

		public:
			PolygonArrayDisplay();
			~PolygonArrayDisplay() override;

			void onInitialize() override;

			void reset() override;

		protected:
			void processMessage(robot_interfaces::msg::PolygonArrayStamped::ConstSharedPtr msg) override;

			std::unique_ptr<rviz_common::properties::ColorProperty> color_property_;
			std::unique_ptr<rviz_common::properties::FloatProperty> alpha_property_;
			std::unique_ptr<rviz_common::properties::EnumProperty> display_mode_property_;

			std::vector<Ogre::ManualObject*> manual_objects_;
			Ogre::MaterialPtr material_;

			std::queue<size_t> removed_id_;
			size_t max_id_;

			void addManualObject();
			void removeManualObject(size_t index);
	};
} // namespace rviz_custom_plugins

#endif // RVIZ_CUSTOM_PLUGINS__POLYGON_ARRAY_DISPLAY_HPP_

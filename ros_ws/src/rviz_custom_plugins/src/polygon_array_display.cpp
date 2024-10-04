// #include <rviz_custom_plugins/polygon_array_display.hpp>
#include <rviz_custom_plugins/polygon_array_display.hpp>

#include "rviz_common/logging.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_rendering/material_manager.hpp"

namespace rviz_custom_plugins
{

	PolygonArrayDisplay::PolygonArrayDisplay()
	{
		color_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
			"Color", QColor(25, 255, 0), "Color to draw the polygons.", this, SLOT(queueRender())
		);

		alpha_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
			"Alpha", 1.0f, "Amount of transparency to apply to the polygons.", this, SLOT(queueRender())
		);
		alpha_property_->setMin(0);
		alpha_property_->setMax(1);

		static int polygon_count = 0;
		std::string material_name = "PolygonArrayMaterial" + std::to_string(polygon_count++);
		material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name);
	}

	PolygonArrayDisplay::~PolygonArrayDisplay()
	{
		if ( initialized() )
		{
			while ( manual_objects_.size() > 0 )
			{
				removeManualObject(0);
			}
		}
	}

	void PolygonArrayDisplay::onInitialize()
	{
		MFDClass::onInitialize();

		max_id_ = 0;
	}

	void PolygonArrayDisplay::reset()
	{
		MFDClass::reset();
		for ( auto& objects : manual_objects_ )
		{
			objects->clear();
		}
	}

	void PolygonArrayDisplay::addManualObject()
	{
		// size_t next_id;
		// if(removed_id_.size() > 0)
		// {
		//     next_id = removed_id_.front();
		//     removed_id_.pop()
		// }
		// else
		// {
		//     next_id = max_id_;
		//     ++max_id_;
		// }

		// Ogre::ManualObject* manual_object {scene_manager_->createManualObject("plane" + std::to_string(next_id))};
		Ogre::ManualObject* manual_object { scene_manager_->createManualObject() };
		manual_object->setDynamic(true);
		scene_node_->attachObject(manual_object);

		manual_objects_.push_back(manual_object);
	}

	void PolygonArrayDisplay::removeManualObject(size_t index)
	{
		if ( index >= manual_objects_.size() )
		{
			return;
		}

		Ogre::ManualObject* manual_object { manual_objects_[index] };
		manual_objects_.erase(manual_objects_.begin() + index);

		scene_node_->detachObject(manual_object);
		scene_manager_->destroyManualObject(manual_object);
	}

	bool validateFloats(robot_interfaces::msg::PolygonArrayStamped::ConstSharedPtr msg)
	{
		bool result { true };

		for ( const auto& polygon : msg->polygons )
		{
			result &= rviz_common::validateFloats(polygon.points);
		}

		return result;
	}

	void PolygonArrayDisplay::processMessage(robot_interfaces::msg::PolygonArrayStamped::ConstSharedPtr msg)
	{
		if ( !validateFloats(msg) )
		{
			setStatus(
				rviz_common::properties::StatusProperty::Error, "Topic",
				"Message contained invalid floating point values (nans or infs)"
			);
			return;
		}

		if ( !updateFrame(msg->header.frame_id, msg->header.stamp) )
		{
			setMissingTransformToFixedFrame(msg->header.frame_id);
			return;
		}
		setTransformOk();

		size_t num_polygons { msg->polygons.size() };

		while ( num_polygons < manual_objects_.size() )
		{
			removeManualObject(0);
		}
		while ( num_polygons > manual_objects_.size() )
		{
			addManualObject();
		}

		Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_property_->getColor());
		color.a = alpha_property_->getFloat();
		rviz_rendering::MaterialManager::enableAlphaBlending(material_, color.a);

		for ( size_t i { 0 }; i < num_polygons; ++i )
		{
			const geometry_msgs::msg::Polygon& polygon { msg->polygons[i] };
			size_t num_points { polygon.points.size() };

			if ( num_points < 3 )
			{
				continue;
			}

			size_t num_triangles { num_points - 2 };
			manual_objects_[i]->clear();
			manual_objects_[i]->estimateVertexCount(2 * num_triangles * 3);
			manual_objects_[i]->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");

			const geometry_msgs::msg::Point32& base_point { polygon.points[0] };
			for ( size_t j { 0 }; j < num_triangles; ++j )
			{
				manual_objects_[i]->position(base_point.x, base_point.y, base_point.z);
				manual_objects_[i]->colour(color);

				manual_objects_[i]->position(polygon.points[j + 1].x, polygon.points[j + 1].y, polygon.points[j + 1].z);
				manual_objects_[i]->colour(color);

				manual_objects_[i]->position(polygon.points[j + 2].x, polygon.points[j + 2].y, polygon.points[j + 2].z);
				manual_objects_[i]->colour(color);

				// Backface Render
				manual_objects_[i]->position(base_point.x, base_point.y, base_point.z);
				manual_objects_[i]->colour(color);

				manual_objects_[i]->position(polygon.points[j + 2].x, polygon.points[j + 2].y, polygon.points[j + 2].z);
				manual_objects_[i]->colour(color);

				manual_objects_[i]->position(polygon.points[j + 1].x, polygon.points[j + 1].y, polygon.points[j + 1].z);
				manual_objects_[i]->colour(color);
			}
			manual_objects_[i]->end();
		}
	}

} // namespace rviz_custom_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_custom_plugins::PolygonArrayDisplay, rviz_common::Display)

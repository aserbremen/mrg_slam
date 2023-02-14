#include <class_loader/class_loader.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <string>
#include <vector>

#define LINKTIME_COMPOSITION_LOGGER_NAME "linktime_composition"

int
main( int argc, char const *argv[] )
{
    // Force flush of the stdout buffer.
    setvbuf( stdout, NULL, _IONBF, BUFSIZ );

    rclcpp::init( argc, argv );
    rclcpp::Logger                                      logger = rclcpp::get_logger( LINKTIME_COMPOSITION_LOGGER_NAME );
    rclcpp::executors::SingleThreadedExecutor           exec;
    rclcpp::NodeOptions                                 options;
    std::vector<class_loader::ClassLoader *>            loaders;
    std::vector<rclcpp_components::NodeInstanceWrapper> node_wrappers;

    std::vector<std::string> libraries = {
        // all classes from libraries linked by the linker (rather then dlopen)
        // are registered under the library_path ""
        "",
    };
    for( auto library : libraries ) {
        RCLCPP_INFO( logger, "Load library %s", library.c_str() );
        auto loader  = new class_loader::ClassLoader( library );
        auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
        for( auto clazz : classes ) {
            RCLCPP_INFO( logger, "Instantiate class %s", clazz.c_str() );
            auto node_factory = loader->createInstance<rclcpp_components::NodeFactory>( clazz );
            auto wrapper      = node_factory->create_node_instance( options );
            auto node         = wrapper.get_node_base_interface();
            node_wrappers.push_back( wrapper );
            exec.add_node( node );
        }
        loaders.push_back( loader );
    }

    exec.spin();

    for( auto wrapper : node_wrappers ) {
        exec.remove_node( wrapper.get_node_base_interface() );
    }
    node_wrappers.clear();

    rclcpp::shutdown();
    return 0;
}

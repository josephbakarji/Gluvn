import argparse
import os
from typing import Optional, Type
from configs.base_config import BaseConfig
from configs.moving_window_config import MovingWindowConfig
from gluvn_python.sens2note_temp import MovingWindow, BaseApp

class AppRunner:
    """Unified runner for all Gluvn applications"""
    
    APP_TYPES = {
        'base': (BaseConfig, BaseApp),
        'moving_window': (MovingWindowConfig, MovingWindow)
    }

    @classmethod
    def create_app(cls, app_type: str, config: BaseConfig):
        """Create an application instance from configuration"""
        if app_type not in cls.APP_TYPES:
            raise ValueError(f"Unknown app type: {app_type}")
        
        config_class, app_class = cls.APP_TYPES[app_type]
        if not isinstance(config, config_class):
            raise ValueError(f"Configuration must be of type {config_class.__name__}")
        
        return app_class(**config.__dict__)

    @classmethod
    def run_app(cls, app_type: str, config_path: Optional[str] = None):
        """Run an application with the specified configuration"""
        # Load configuration
        if config_path:
            config_class = cls.APP_TYPES[app_type][0]
            config = config_class.from_file(config_path)
        else:
            config_class = cls.APP_TYPES[app_type][0]
            config = config_class()

        # Create and run application
        app = cls.create_app(app_type, config)
        app.start()

        try:
            key = input('Press any key to finish\n')
            print('Shutting down...')
        except KeyboardInterrupt:
            print('\nShutting down...')
        finally:
            app.reader.stop_readers()
            for hand in app.hands:
                app.triggers[hand].join(timeout=.1)
            app.join(timeout=.1)

def main():
    parser = argparse.ArgumentParser(description='Run Gluvn applications with configuration')
    parser.add_argument('--app', type=str, required=True, choices=['base', 'moving_window'],
                      help='Type of application to run')
    parser.add_argument('--config', type=str, help='Path to configuration JSON file')
    parser.add_argument('--save-config', type=str, help='Path to save current configuration')
    args = parser.parse_args()

    if args.save_config:
        config_class = AppRunner.APP_TYPES[args.app][0]
        config = config_class()
        config.to_file(args.save_config)
        print(f"Configuration saved to {args.save_config}")
        return

    AppRunner.run_app(args.app, args.config)

if __name__ == "__main__":
    main() 
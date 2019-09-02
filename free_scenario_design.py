"""
Welcome to CARLA manual control.
Use keys and mouse wheel for control.

-----------------------------------------------------------------------------------
camera manager:
    W            : upward
    S            : downward
    AD           : left and right

    MouseWheelDown: zoom backward
    MouseWheelUp  : zoom forward

    TAB          : Change camera,by click TAB, you change your camera to two views.
-----------------------------------------------------------------------------------
object manager:
    Mouseleftdown  : Change object to place,and preview it.
    Mouserightdown : Place the object

    BACKSPACE    : Delete the last placed object
-----------------------------------------------------------------------------------
routes manager:
    Mouseleftdown  : 
    Mouserightdown : 
-----------------------------------------------------------------------------------
    C            : you can press c to change the mode to route or object set.
    R            : Record the object placed into a xml style file for scenario_runner

    shift + R    : Restart
    H/?          : toggle help
    ESC          : quit
"""


import carla
from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

import numpy as np
from line_fitting_helpers import catmull_rom_spline as cftool


try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_u, K_i, K_o, K_j, K_k, K_l
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError(
        'cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

OBJECT_FILE = "my_config.csv"
ROUTES_FILE = "my_routes.csv"


class World(object):
    def __init__(self, carla_world, hud, actor_filter, actor_role_name='hero'):
        self.world = carla_world
        self.actor_role_name = actor_role_name
        self.map = self.world.get_map() #return a Map
        self.hud = hud
        self.player = None
        self.camera_manager = None
        self.object_manager = None
        self.routes_manager = None
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0

        

    def restart(self):
        #set up or clean up the object manager
        if self.object_manager is None:
            self.object_manager = ObjectManager(self)
        else:
            self.object_manager.__del__()
        #set up or clean up the routes manager
        if self.routes_manager is None:
            self.routes_manager = RoutesManager(self)
        else:
            self.routes_manager.__del__()
        
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        # Spawn the player.
        # Set up the sensors.
        self.camera_manager = CameraManager(
            self.world, self.hud)  # change to self.world
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = "camera" + str(cam_pos_index)
        self.hud.notification(actor_type)

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        actors = [
            self.camera_manager.sensor]
        for actor in actors:
            if actor is not None:
                actor.destroy()
        self.object_manager.__del__()
        self.routes_manager.__del__()

# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    def __init__(self, world, start_in_autopilot=False):
        self._autopilot_enabled = False
        self._steer_cache = 0.0
        self.mode = 'set_objects'
        self.modes = ['set_objects', 
                    'set_routes']
        self.mode_index = 0

        self.waypoint_generate = 'autonomous'
        self.waypoint_generates = ['autonomous',
                                   'hand']
        self.generate_index = 0
        self.world = world
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def change_mode(self):
        self.mode_index = (self.mode_index + 1) % len(self.modes)
        self.mode = self.modes[self.mode_index]
        self.world.object_manager.delete_preview_last_object()
        self.world.routes_manager.delete_preview_last_object()
        self.world.hud.notification("change mode to {}".format(self.mode), seconds=4.0)

    def change_waypoint_generate(self):
        self.generate_index = (self.generate_index + 1) % len(self.waypoint_generates)
        self.waypoint_generate = self.waypoint_generates[self.generate_index]
        self.world.hud.notification("change waypoint generate way to {}".format(self.waypoint_generate),seconds=4.0)

    def parse_events(self, client, world, clock):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_r and pygame.key.get_mods() & KMOD_SHIFT:
                    world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r:
                    world.object_manager.save_object_configure()
                    world.routes_manager.save_routes_configure()
                elif event.key == K_u:
                    world.object_manager.change_object()
                elif event.key == K_BACKSPACE:
                    world.object_manager.delete_last_object()
                elif event.key == K_c:
                    self.change_mode()
                elif event.key == K_o:
                    self.change_waypoint_generate()
                elif event.key == K_i:
                    world.routes_manager.fit_routes()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                pressed_array = pygame.mouse.get_pressed()
                for index in range(len(pressed_array)):
                    if self.mode == 'set_objects':
                        if pressed_array[index]:
                            if index == 0:
                                world.object_manager.change_object()
                            elif index == 2:
                                world.object_manager.try_add_object()  
                    elif self.mode == 'set_routes':
                        if pressed_array[index]: 
                            if index == 0: 
                                world.routes_manager.preview_waypoints_tools()
                            elif index == 2:
                                if self.waypoint_generate == 'autonomous':
                                    world.routes_manager.try_add_waypoints()
                                elif self.waypoint_generate == 'hand':
                                    world.routes_manager.try_add_your_points()
            world.camera_manager.parse_camera_control(event)
    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        fonts = [x for x in pygame.font.get_fonts() if 'mono' in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.server_fps = 0
        self.frame_number = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

        self.start_recording = False
        self.current_point = None

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame_number = timestamp.frame_count
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.camera_manager.on_ground_camera_transform
        object_name = world.object_manager.object_name
        object_num = len(world.object_manager.objects)
        current_mode_object_route = world.controller.mode
        current_route_setting_mode = world.controller.waypoint_generate
        heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > t.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > t.rotation.yaw > -179.5 else ''
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Map:     % 20s' % world.map.name,
            'Simulation time: % 12s' % datetime.timedelta(
                seconds=int(self.simulation_time)),
            '',
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (t.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' %
                                (t.location.x, t.location.y)),
            'Height:  % 18.0f m' % t.location.z,
            "",
        ]
        if current_mode_object_route == "set_objects":
            self._info_text += [
            "Current object:% 10s" %object_name,
            "number of objects %5.0f" %object_num,
            ""
            ]
        if current_mode_object_route == "set_routes":
            self._info_text += [
            "Current route_setting_mode:",
            current_route_setting_mode,
            ""
            ]

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30)
                                  for x, y in enumerate(item)]
                        pygame.draw.lines(
                            display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect(
                            (bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255),
                                         rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect(
                            (bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(
                            display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect(
                                (bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect(
                                (bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(
                        item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 *
                    self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================

class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False

        self.on_ground_camera_transform = carla.Transform(
            carla.Location(x=10, z=2.8), carla.Rotation(pitch= -15)
        )
        self.high_camera_transform = carla.Transform(
            carla.Location(x=10, z=100), carla.Rotation(pitch=-90)
        )
        self._camera_transforms = [
            self.on_ground_camera_transform,
            self.high_camera_transform
        ]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB']
        ]
        world = parent_actor
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '5000')
            item.append(bp)
        self.index = None

        self.linear_speed = 1
        self.angular_speed = 4

    def parse_camera_control(self, event):
        if event.type == pygame.KEYDOWN:
            event_key = event.key
            if self.transform_index == 0:
                if event_key == K_w:
                    self.on_ground_camera_transform.location.z += self.linear_speed * 0.3
                elif event_key == K_s:
                    self.on_ground_camera_transform.location.z -= self.linear_speed * 0.3
                elif event_key == K_a:
                    self.on_ground_camera_transform.rotation.yaw -= self.angular_speed
                elif event_key == K_d:
                    self.on_ground_camera_transform.rotation.yaw += self.angular_speed
                else:
                    return

            elif self.transform_index == 1:
                if event_key == K_w:
                    self.high_camera_transform.location.x += self.linear_speed * 5
                elif event_key == K_s:
                    self.high_camera_transform.location.x -= self.linear_speed * 5
                elif event_key == K_a:
                    self.high_camera_transform.location.y -= self.linear_speed * 5
                elif event_key == K_d:
                    self.high_camera_transform.location.y += self.linear_speed * 5
                else:
                    return

            self._camera_transforms = [
                self.on_ground_camera_transform,
                self.high_camera_transform
            ]
            self.sensor.set_transform(
                self._camera_transforms[self.transform_index])

        elif event.type == pygame.MOUSEBUTTONDOWN:
            is_changed = True
            if self.transform_index == 0:
                if event.button == 4:
                    yaw = self.on_ground_camera_transform.rotation.yaw * np.pi / 180
                    self.on_ground_camera_transform.location.x += self.linear_speed * \
                        np.cos(yaw)
                    self.on_ground_camera_transform.location.y += self.linear_speed * \
                        np.sin(yaw)
                elif event.button == 5:
                    yaw = self.on_ground_camera_transform.rotation.yaw * np.pi / 180
                    self.on_ground_camera_transform.location.x -= self.linear_speed * \
                        np.cos(yaw)
                    self.on_ground_camera_transform.location.y -= self.linear_speed * \
                        np.sin(yaw)
                else:
                    return
            if self.transform_index == 1:

                if event.button == 4:
                    self.high_camera_transform.location.z += self.linear_speed * 5
                elif event.button == 5:
                    self.high_camera_transform.location.z -= self.linear_speed * 5
                else:
                    return
            
            self._camera_transforms = [
                self.on_ground_camera_transform,
                self.high_camera_transform
            ]
            self.sensor.set_transform(
                self._camera_transforms[self.transform_index])

    def toggle_camera(self):
        self.transform_index = (self.transform_index +
                                1) % len(self._camera_transforms)
        self.sensor.set_transform(
            self._camera_transforms[self.transform_index])

    def set_sensor(self, index, notify=True):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None \
            else self.sensors[index][0] != self.sensors[self.index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(
                lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' %
                              ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 3), 3))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / 100.0
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame_number)

# ==============================================================================
# -- add your own obstacle -----------------------------------------------------
# ==============================================================================

class ObjectManager(object):
    def __init__(self, world):
        global OBJECT_FILE
        assert(isinstance(world, World))
        self.carla_world = world.world
        self.hud = world.hud
        self.objects = []
        self.preview_objects = []
        self.World = world
        self.object_names = ["static.prop.constructioncone",
                             "static.prop.box01",
                             "static.prop.trafficcone02",
                             "chainbarrierend"]

        self.object_index = 0
        self.object_name =  self.object_names[self.object_index]
        self.config_file = OBJECT_FILE
        
    def preview_add_object(self):
        current_transform = self.World.camera_manager.on_ground_camera_transform
        output_transform = carla.Transform()
        output_transform.location.x = 10 
        output_transform.location.y = 0 
        output_transform.location.z = 0
        output_transform.rotation.pitch = -current_transform.rotation.pitch 
 
        if self.object_name == "static.prop.clothesline":
            output_transform.rotation.yaw += 90
             
            
        blueprint = random.choice(self.carla_world.get_blueprint_library().filter(self.object_name))
        actor = self.carla_world.try_spawn_actor(blueprint, output_transform,attach_to = self.World.camera_manager.sensor)
        if actor is not None:
            self.preview_objects.append(actor)
            self.hud.notification("preview the object you select {}".format(self.object_name))
            actor.set_simulate_physics(enabled=False)
        else:
            self.hud.notification("preview {} object failed".format(self.object_name))


    def try_add_object(self):
        if len(self.preview_objects) > 0:
            object_ = self.preview_objects[len(self.preview_objects)-1]
            output_transform = object_.get_transform()
            self.delete_preview_last_object()
            blueprint = random.choice(self.carla_world.get_blueprint_library().filter(self.object_name))
            actor = self.carla_world.try_spawn_actor(blueprint, output_transform)
            if actor is not None:
                self.objects.append(actor)
                self.hud.notification("add one object {}".format(self.object_name))
                actor.set_simulate_physics(enabled=True)
        else:
            self.hud.notification("add {} object failed,please click the leftmouse to preview".format(self.object_name))

    def delete_last_object(self):
        if len(self.objects) > 0:
            object_ = self.objects.pop()
            location = object_.get_location()
            self.hud.notification("delete one object, remaining {} objects".format(str(len(self.objects))))
            self.hud.notification("the deleted object ,locates in {}, {}".format(
                str(location.x),str(location.y)
            ))
            
            object_.destroy()

    def delete_preview_last_object(self):
        if len(self.preview_objects) > 0:
            object_ = self.preview_objects.pop()
            location = object_.get_location()
            #self.hud.notification("delete one object, remaining {} objects".format(str(len(self.preview_objects))))
            #self.hud.notification("the deleted object ,locates in {}, {}".format(
            #    str(location.x),str(location.y)
            #))
            
            object_.destroy()    

    def save_object_configure(self):
        with open(self.config_file, "w") as my_file:
            lines = []
            init_text = "x,y,z,pitch,roll,yaw,object_name\n"
            my_file.write(init_text)
            for object_ in self.objects:
                transform = object_.get_transform()
                object_type = str(object_).split("type=")[1][:-1]
                x = transform.location.x
                y = transform.location.y
                z = transform.location.z
                pitch = transform.rotation.pitch
                roll = transform.rotation.roll
                yaw = transform.rotation.yaw
                text = "{},{},{},{},{},{},{}\n".format(
                    str(x), str(y), str(z), str(pitch), str(roll), str(yaw), object_type
                )
                lines.append(text)
            my_file.writelines(lines)

        self.hud.notification("save {} objects config in {}".format(
            str(len(self.objects)), self.config_file))
    
    def change_object(self):
        if len(self.preview_objects) > 0:
            self.delete_preview_last_object()
            self.object_index = (self.object_index + 1) % len(self.object_names)
            self.object_name = self.object_names[self.object_index]
        self.preview_add_object()

    def __del__(self):
        for i in range(len(self.objects)):
            object_ = self.objects.pop()
            object_.destroy()
        for j in range(len(self.preview_objects)):
            object_ = self.preview_objects.pop()
            object_.destroy()

# ==============================================================================
# -- set your own simulating routes by using waypoint in carla -----------------
# ==============================================================================

class RoutesManager(object):
    def __init__(self, world):
        global ROUTES_FILE                         # myconfig.config 
        assert(isinstance(world, World))           # prove that world is example of World.
        self.carla_world = world.world             # world initial from the class World, which includes world
        self.hud = world.hud
        self.waypoints_transforms = []
        self.waypoints_interpolate_transforms = []
        self.preview_tools = []
        self.World = world
        self.waypoint_index = 0
        self.map = world.map
        self.debuger = world.world.debug
        self.object_name = "static.prop.wateringcan"
        self.config_file = ROUTES_FILE
        self.debugcolor = carla.Color(r=0,b=0,g=0)
        self.res = 8   
        
    
    def fit_routes(self):
        if len(self.waypoints_transforms) > 0:
            waypoints_transform_x = []
            waypoints_transform_y = []
            for i in range(len(self.waypoints_transforms)):
                waypoints_transform_x.append(self.waypoints_transforms[i].location.x)
                waypoints_transform_y.append(self.waypoints_transforms[i].location.y)
            x, y = cftool.catmull_rom(waypoints_transform_x, waypoints_transform_y,self.res)
            transform = []
            for j in range(len(x)):
                transform = carla.Transform()
                transform.location.x = x[j]
                transform.location.y = y[j]
                zwischen_waypoint = self.map.get_waypoint(transform.location,project_to_road = True, lane_type = carla.LaneType.Driving)
                transform.location.z = zwischen_waypoint.transform.location.z
                transform.rotation = zwischen_waypoint.transform.rotation
                self.waypoints_interpolate_transforms.append(transform)
                self.debuger.draw_point(transform.location,size = 0.07, life_time = 36000.0)        
            for j in range(len(self.waypoints_interpolate_transforms)):
                print("x:",self.waypoints_interpolate_transforms[j].location.x)
                print("y:",self.waypoints_interpolate_transforms[j].location.y)
                print("z:",self.waypoints_interpolate_transforms[j].location.z)


        
        
    def preview_waypoints_tools(self):
        if len(self.preview_tools) > 0:
             object_ = self.preview_tools.pop()
             object_.destroy()
        output_transform = carla.Transform()
        output_transform.location.x = 4
        output_transform.location.y = 0 
        output_transform.location.z = 0
        output_transform.rotation.pitch += -30 
        output_transform.rotation.yaw += 90
        output_transform.rotation.roll += -1
        blueprint = random.choice(self.carla_world.get_blueprint_library().filter(self.object_name))
        actor = self.carla_world.try_spawn_actor(blueprint, output_transform,attach_to = self.World.camera_manager.sensor)
        if actor is not None:
            self.preview_tools.append(actor)
            self.hud.notification("now you can choose your starting waypoint by using the tool {},you have 1 hour to choose you routes.".format(actor.__str__()),seconds=30.0)
            print(actor.__str__())
            actor.set_simulate_physics(enabled=False)
        else:
            self.hud.notification("fail to add preview_tools {}".format(self.object_name))
        
      
    def try_add_your_points(self):
        if len(self.preview_tools) > 0:
            object_ = self.preview_tools[len(self.preview_tools)-1]
            output_transform = object_.get_transform()
            your_now_waypoint = self.map.get_waypoint(output_transform.location,project_to_road = True, lane_type = carla.LaneType.Driving)
            output_transform.location.z = your_now_waypoint.transform.location.z
            self.debuger.draw_point(output_transform.location,size = 0.15,color=self.debugcolor, life_time = 36000.0)
            if your_now_waypoint is not None:
                self.waypoints_transforms.append(output_transform)
                self.hud.notification("add one point in x {},y {},z {},if you do not want to add,press i to fit your routes"\
                    .format(your_now_waypoint.transform.location.x,your_now_waypoint.transform.location.y,your_now_waypoint.transform.location.z),seconds = 4.0)
        else:
            self.hud.notification("add {} your_points failed,please click the leftmouse to preview the tools")
          
        

    def try_add_waypoints(self):
        if len(self.preview_tools) > 0:
            object_ = self.preview_tools[len(self.preview_tools)-1]
            output_transform = object_.get_transform()
            your_now_waypoint = self.map.get_waypoint(output_transform.location,project_to_road = True, lane_type = carla.LaneType.Driving)
            self.debuger.draw_point(your_now_waypoint.transform.location,size = 0.15,color=self.debugcolor, life_time = 36000.0)
          
            if your_now_waypoint is not None:
                self.waypoints_transforms.append(your_now_waypoint.transform)
                self.hud.notification("add one point in x {},y {},z {},if you do not want to add,press m to fit your routes"\
                    .format(your_now_waypoint.transform.location.x,your_now_waypoint.transform.location.y,your_now_waypoint.transform.location.z),seconds = 4.0)
        else:
            self.hud.notification("add {} way_points failed,please click the leftmouse to preview the tools")

    # Because the mark cannot be push, so this might a problem
    
    def delete_preview_last_object(self):
        if len(self.preview_tools) > 0:
            object_ = self.preview_tools.pop()
            location = object_.get_location()
            self.hud.notification("preview actor get destoyed")
            
            object_.destroy()  


    def save_routes_configure(self):
        with open(self.config_file, "w") as my_file:
            lines = []
            init_text = "x,y,z,yaw\n"
            my_file.write(init_text)
            for transform in self.waypoints_interpolate_transforms:
                x = transform.location.x
                y = transform.location.y
                z = transform.location.z
                yaw = transform.rotation.yaw
                text = "{},{},{},{}\n".format(
                    str(x), str(y), str(z),  str(yaw) 
                )
                lines.append(text)
            my_file.writelines(lines)

        self.hud.notification("save {} ROUTES config in {}".format(
            str(len(self.waypoints_interpolate_transforms)), self.config_file))

    def __del__(self):
        for i in range(len(self.preview_tools)):
            object_ = self.preview_tools.pop()
            object_.destroy()
        self.waypoints_transforms = []
        self.waypoints_interpolate_transforms = []
        
# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        debuger = carla.DebugHelper
        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        
        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args.filter, args.rolename)
        controller = KeyboardControl(world, args.autopilot)
        world.controller = controller

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock):
                return
            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)
    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()

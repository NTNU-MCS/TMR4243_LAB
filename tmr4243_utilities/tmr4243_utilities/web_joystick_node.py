#!/usr/bin/env python3

import argparse
import asyncio
import contextlib
import json
import os
import signal
import threading
import functools
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


def _package_root() -> Path:
	# tmr4243_utilities/tmr4243_utilities/web_joystick.py -> tmr4243_utilities/
	return Path(__file__).resolve().parents[1]


def _public_dir() -> Path:
	return _package_root() / 'public'


class _HttpServerThread(threading.Thread):
	def __init__(self, directory: Path, host: str, port: int):
		super().__init__(daemon=True)
		self._directory = directory
		self._host = host
		self._port = port
		self._httpd: Optional[ThreadingHTTPServer] = None

	def run(self) -> None:
		handler = functools.partial(SimpleHTTPRequestHandler, directory=str(self._directory))
		self._httpd = ThreadingHTTPServer((self._host, self._port), handler)
		self._httpd.serve_forever(poll_interval=0.25)

	def shutdown(self) -> None:
		if self._httpd is not None:
			self._httpd.shutdown()


class WebJoyBridge(Node):
	def __init__(self, topic: str = '/joy'):
		super().__init__('web_joystick')
		self._pub = self.create_publisher(Joy, topic, 10)
		self._last_msg: Optional[Joy] = None

	def publish_from_payload(self, payload: Dict[str, Any]) -> None:
		axes = payload.get('axes', [])
		buttons = payload.get('buttons', [])
		button_values = payload.get('button_values', None)

		if not isinstance(axes, list) or not isinstance(buttons, list):
			return

		msg = Joy()
		msg.header.stamp = self.get_clock().now().to_msg()

		try:
			msg_axes = [float(v) for v in axes]
		except (TypeError, ValueError):
			msg_axes = []

		# Many gamepads expose triggers as analog button values (0..1)
		# rather than axes. To make those available to downstream ROS nodes
		# without changing Joy.buttons semantics, append them to Joy.axes.
		if isinstance(button_values, list):
			try:
				msg_axes.extend([float(v) for v in button_values])
			except (TypeError, ValueError):
				pass

		msg.axes = msg_axes

		try:
			msg.buttons = [int(v) for v in buttons]
		except (TypeError, ValueError):
			msg.buttons = []

		self._pub.publish(msg)
		self._last_msg = msg


async def _ws_server(
	node: WebJoyBridge,
	host: str,
	port: int,
	path: str = '/ws',
) -> None:
	try:
		import websockets
	except Exception as exc:  # pragma: no cover
		raise RuntimeError(
			"Missing dependency 'websockets'. Install with: pip install websockets"
		) from exc

	async def handler(ws: Any) -> None:
		async for message in ws:
			try:
				payload = json.loads(message)
			except json.JSONDecodeError:
				continue
			if isinstance(payload, dict):
				node.publish_from_payload(payload)

	def _get_request_path(ws: Any, request_path: Any) -> str:
		# websockets has had multiple handler signatures over versions.
		# - legacy: handler(websocket, path)
		# - newer: handler(connection) with path available on the connection
		if isinstance(request_path, str):
			return request_path
		# best-effort fallbacks
		p = getattr(ws, 'path', None)
		if isinstance(p, str):
			return p
		req = getattr(ws, 'request', None)
		p = getattr(req, 'path', None)
		if isinstance(p, str):
			return p
		return ''

	async def router(ws: Any, request_path: Any = None) -> None:
		resolved_path = _get_request_path(ws, request_path)
		if resolved_path != path:
			await ws.close(code=1008, reason='Invalid websocket path')
			return
		await handler(ws)

	async with websockets.serve(router, host, port, ping_interval=20, ping_timeout=20):
		node.get_logger().info(f"WebSocket listening on ws://{host}:{port}{path}")
		await asyncio.Future()  # run forever


def _parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description='WebSocket -> ROS2 Joy bridge', add_help=True)
	parser.add_argument('--http-host', default='127.0.0.1')
	parser.add_argument('--http-port', type=int, default=8000)
	parser.add_argument('--ws-host', default='127.0.0.1')
	parser.add_argument('--ws-port', type=int, default=8765)
	parser.add_argument('--ws-path', default='/ws')
	parser.add_argument('--topic', default='/joy')
	# When launched via ros2 launch, ROS injects arguments like:
	#   --ros-args -r __node:=...
	# Accept and ignore unknown args so argparse doesn't exit.
	known, _unknown = parser.parse_known_args(sys.argv[1:])
	return known


def main(args: Optional[List[str]] = None) -> None:
	cli = _parse_args()

	public_dir = _public_dir()
	if not public_dir.exists():
		raise FileNotFoundError(f"Public directory not found: {public_dir}")

	# Make sure relative file serving works even if launched elsewhere.
	os.chdir(str(public_dir))

	rclpy.init(args=args)
	node = WebJoyBridge(topic=cli.topic)

	http_thread = _HttpServerThread(public_dir, cli.http_host, cli.http_port)
	http_thread.start()
	node.get_logger().info(f"HTTP serving {public_dir} at http://{cli.http_host}:{cli.http_port}/joystick.html")

	loop = asyncio.new_event_loop()
	asyncio.set_event_loop(loop)

	stop_event = asyncio.Event()

	def _stop(*_: Any) -> None:
		try:
			loop.call_soon_threadsafe(stop_event.set)
		except RuntimeError:
			pass

	for sig in (signal.SIGINT, signal.SIGTERM):
		try:
			signal.signal(sig, _stop)
		except Exception:
			pass

	async def runner() -> None:
		ws_task = asyncio.create_task(_ws_server(node, cli.ws_host, cli.ws_port, path=cli.ws_path))
		await stop_event.wait()
		ws_task.cancel()
		with contextlib.suppress(asyncio.CancelledError):
			await ws_task

	try:
		loop.run_until_complete(runner())
	finally:
		http_thread.shutdown()
		node.destroy_node()
		rclpy.shutdown()
		try:
			loop.stop()
			loop.close()
		except Exception:
			pass


if __name__ == '__main__':
	main()

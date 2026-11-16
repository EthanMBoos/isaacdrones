"""See isaacdrones's GUI on your own computer while Isaac Sim runs on PACE ICE.

On your own computer, run the line that `scripts/pace.sh stream` prints:

    python3 scripts/stream_relay.py connect USERNAME NODE

It logs in to ICE (password and Duo as usual), then tells you to open the Isaac
Sim WebRTC Streaming Client and connect to 127.0.0.1. Ctrl-C ends it.

Isaac Sim streams its window as WebRTC video over UDP, and ICE admits only SSH
from outside. An SSH tunnel carries only TCP. `connect` therefore opens a tunnel
and wraps the UDP video in TCP, and `node`, which `scripts/pace.sh stream`
starts beside Isaac Sim, unwraps it. See docs/PACE.md.

    connect  UDP 127.0.0.1:47998 (the client) <-> TCP 47999, through the tunnel
    node     TCP 47999 <-> UDP 127.0.0.1:47998 (Isaac Sim)

Each datagram crosses TCP as a 2-byte length followed by its bytes.
"""

import socket
import struct
import subprocess
import sys
import threading
import time

UDP_PORT, TCP_PORT = 47998, 47999


def recv_exact(tcp: socket.socket, n: int) -> bytes:
    data = b""
    while len(data) < n:
        chunk = tcp.recv(n - len(data))
        if not chunk:
            raise ConnectionError("TCP side closed")
        data += chunk
    return data


def pump(udp: socket.socket, tcp: socket.socket, udp_peer) -> None:
    """Copy both ways until TCP closes. `udp_peer` is where UDP goes, or None to
    reply to whoever sent the last datagram."""
    peer = [udp_peer]

    def udp_to_tcp() -> None:
        while True:
            data, sender = udp.recvfrom(65535)
            if udp_peer is None:
                peer[0] = sender
            tcp.sendall(struct.pack("!H", len(data)) + data)

    threading.Thread(target=udp_to_tcp, daemon=True).start()
    while True:
        (length,) = struct.unpack("!H", recv_exact(tcp, 2))
        data = recv_exact(tcp, length)
        if peer[0] is not None:
            udp.sendto(data, peer[0])


def main() -> None:
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    if mode == "node":
        server = socket.socket()
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", TCP_PORT))
        server.listen()
        while True:
            tcp, sender = server.accept()
            tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"stream_relay: connection from {sender[0]}", flush=True)
            try:
                pump(socket.socket(socket.AF_INET, socket.SOCK_DGRAM), tcp, ("127.0.0.1", UDP_PORT))
            except ConnectionError:
                print("stream_relay: connection closed", flush=True)
    elif mode == "connect" and len(sys.argv) == 4:
        connect(username=sys.argv[2], node=sys.argv[3])
    else:
        sys.exit(__doc__)


def connect(username: str, node: str) -> None:
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        udp.bind(("127.0.0.1", UDP_PORT))
    except OSError:
        sys.exit(f"UDP port {UDP_PORT} is in use. Is another stream_relay.py still running?")

    # 49100 is the client's own connection to Isaac Sim; 47999 carries the video.
    print("Logging in to ICE. Enter your password and approve Duo if asked.", flush=True)
    tunnel = subprocess.Popen(
        ["ssh", "-N", "-o", "ExitOnForwardFailure=yes"]
        + ["-L", f"49100:{node}:49100", "-L", f"{TCP_PORT}:{node}:{TCP_PORT}"]
        + [f"{username}@login-ice.pace.gatech.edu"]
    )
    try:
        tcp = None
        while tcp is None:
            if tunnel.poll() is not None:
                sys.exit("The SSH tunnel closed. Check the username, the VPN and the node name.")
            try:
                tcp = socket.create_connection(("127.0.0.1", TCP_PORT), timeout=2)
            except OSError:
                time.sleep(1)
        tcp.settimeout(None)
        tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(
            "\nConnected. Open the Isaac Sim WebRTC Streaming Client, type 127.0.0.1\n"
            "as the server and click Connect. Leave this running; Ctrl-C ends it.",
            flush=True,
        )
        pump(udp, tcp, None)
    except (ConnectionError, KeyboardInterrupt):
        print("\nStream ended. If the job on ICE is still running: scancel JOBID")
    finally:
        tunnel.terminate()


if __name__ == "__main__":
    main()

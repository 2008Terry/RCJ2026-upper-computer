#!/usr/bin/env python3

from telemetry_image_compression import spin_image_compression_relay


def main(args=None):
    spin_image_compression_relay(node_name="image_compression_relay", args=args)


if __name__ == "__main__":
    main()

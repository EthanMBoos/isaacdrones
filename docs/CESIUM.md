# Cesium

[Cesium for Omniverse](https://github.com/CesiumGS/cesium-omniverse) streams
real places into Isaac Sim as 3D tiles, including Google's photorealistic tiles.
Release 0.30.0 runs in Isaac Sim 6.2, headless on ICE, and has rendered the
Georgia Tech campus through the RTX camera. It is not a `--scene` yet;
[TODO.md](TODO.md) has that step. This page covers what a person must set up by
hand first.

## A Cesium ion token

Cesium serves the tiles only to an account, and the account is free.

1. Sign up at <https://ion.cesium.com/signup>.
2. In Cesium ion, open **Access Tokens** and copy the default token.
3. In **Asset Depot**, add **Google Photorealistic 3D Tiles** to the account
   if it is not already under **My Assets**.
4. Save the token on ICE in a file that only its owner can read:

```bash
cat > ~/.cesium_token     # paste the token, press Enter, then Ctrl-D
chmod 600 ~/.cesium_token
```

The token is a secret. It stays in that file and is passed to Isaac Sim through
the environment, as `CESIUM_ION_TOKEN`. It is never written into this
repository, a script or a log.

To check the token from a login node:

```bash
curl -s -H "Authorization: Bearer $(cat ~/.cesium_token)" https://api.cesium.com/v1/me
```

A working token prints the account's id and its scopes, which must include
`assets:read`.

## What Cesium needs from Isaac Sim

Cesium needs three things that the other scenes do not. It runs in Isaac Sim's
stock app, because it imports GUI extensions that
`src/isaacdrones/sim/isaacdrones.kit` leaves out; the stock app takes from 13 s
to several minutes to start, depending on whether the node has read its files
before. It must be enabled when the app starts, with `--ext-folder` and
`--enable`, because it registers USD schemas. And it needs the `lxml` package,
which it cannot install for itself in this environment.

Google's tiles are asset 2275207 in Cesium ion. Google requires its credit to be
shown with the imagery, and its terms need reading before anyone records data
from the tiles or trains on it.

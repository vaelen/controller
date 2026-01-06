# SatNOGS Database API

[SatNOGS](https://satnogs.org/) is an open source global network of satellite ground-stations. As part of the project they maintain a [database](https://db.satnogs.org/) of publically available satellite information. This database hosts a machine readable [API](https://db.satnogs.org/api/). The [API definition](https://librespacefoundation.gitlab.io/satnogs/satnogs-network/api/openapi.yml) is maintained in the [OpenAPI](https://www.openapis.org/) format. ([OpenAPI Specification](https://spec.openapis.org/oas/latest.html)) An [interactive version](https://librespacefoundation.gitlab.io/satnogs/satnogs-network/api/index.html) of the API documentation is also available.

## Communication Channel Information

The Sattrack ground station controller can access the SatNOGS "transmitters" API to gather information about the communication channels offered by various satellites. It is a simple REST API.

### AuthenticationRX and 1 TX communication channel defined.

For any client to authenticate to the SatNOGS APIs, they must pass the `Authorization` header with the value set to an active API key. To use the SatNOGS APIs from within the ground station controller, you must provide your own API key in the controller configuration file. The website explains how to get one. You will need to sign up for a free SatNOGS account if you do not already have one.

### List Transmitters

```http
GET /api/transmitters/
```

This returns the list of all transmitters. Transmitters are referenced by a SatNOGS internally defined UUID.

### Get Transmitter Detail

```http
GET /api/transmitters/{transmitter_uuid}
```

This returns the details for a given transmitter. A SatNOGS transmitter represents the transmitter on a satellite and each transmitter can have both a transmit (downlink) and receive (uplink) channel defined.

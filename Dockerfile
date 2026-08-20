FROM python:3.11-slim

COPY app /app
RUN python -m pip install /app --extra-index-url https://www.piwheels.org/simple

EXPOSE 8000/tcp
EXPOSE 14560/udp

LABEL version="0.1.0"

ARG IMAGE_NAME

LABEL permissions='\
{\
  "ExposedPorts": {\
    "8000/tcp": {},\
    "14560/udp": {}\
  },\
  "HostConfig": {\
    "ExtraHosts": ["host.docker.internal:host-gateway"],\
    "Devices": [\
      {\
        "PathOnHost": "/dev/ttyUSB0",\
        "PathInContainer": "/dev/ttyUSB0",\
        "CgroupPermissions": "rwm"\
      }\
    ],\
    "PortBindings": {\
      "8000/tcp": [\
        {\
          "HostPort": ""\
        }\
      ],\
      "14560/udp": [\
        {\
          "HostPort": "14560"\
        }\
      ]\
    }\
  }\
}'

ARG AUTHOR
ARG AUTHOR_EMAIL
LABEL authors='[\
    {\
        "name": "$AUTHOR",\
        "email": "$AUTHOR_EMAIL"\
    }\
]'

ARG MAINTAINER
ARG MAINTAINER_EMAIL
LABEL company='{\
    "about": "",\
    "name": "$MAINTAINER",\
    "email": "$MAINTAINER_EMAIL"\
}'

LABEL type="device-integration"

ARG REPO
ARG OWNER

LABEL readme='https://raw.githubusercontent.com/$OWNER/$REPO/{tag}/README.md'
LABEL links='{\
    "source": "https://github.com/$OWNER/$REPO"\
}'

LABEL requirements="core >= 1.1"

ENTRYPOINT ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000", "--app-dir", "/app"]

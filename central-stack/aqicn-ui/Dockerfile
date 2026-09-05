# Built on the Pi itself (aarch64), so keep the build cheap: no cgo, no module
# downloads — the service is stdlib-only on purpose.
FROM golang:1.24-alpine AS build

WORKDIR /src
COPY . .
RUN CGO_ENABLED=0 go build -trimpath -ldflags="-s -w" -o /aqicn-ui .

# The binary makes no outbound connections and reads no system files, so it
# needs neither TLS roots nor a timezone database. scratch keeps the image at
# roughly the size of the binary.
FROM scratch
COPY --from=build /aqicn-ui /aqicn-ui
EXPOSE 8080
ENTRYPOINT ["/aqicn-ui"]

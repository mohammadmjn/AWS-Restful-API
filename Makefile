.PHONY: build clean deploy

.EXPORT_ALL_VARIABLES:
AWS_REGION ?= us-east-1
TABLE_NAME ?= aws-challenge-devices-dev

build:
	dep ensure -v
	env GOOS=linux go build -ldflags="-s -w" -o bin/postDevice postDevice/post.go
	env GOOS=linux go build -ldflags="-s -w" -o bin/getDevice getDevice/get.go

clean:
	rm -rf ./bin ./vendor Gopkg.lock

deploy: clean build
	sls deploy --verbose

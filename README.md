# A Simple Golang AWS Restful API

This package, which is developed in Golang, implements two simple HTTP request (POST, GET) on Amazon Web Services (AWS) in order to put an item in DynamoDB table through POST request, as well as fetch information about and specific item from DynamoDB table (if exists) via GET request. The package applies AWS Lambda function to interact with AWS interface.

## API Request Type

The API accepts the following JSON requests and produces the corresponding HTTP responses:

### Request 1

POST request to insert a new device to DynamoDB database

```
HTTP Method: POST
URL: https://<api-gateway-url>/api/devices
content-type: application/json
Body:
  {
    "id": "/devices/id1",
    "deviceModel": "/devicemodels/id1",
    "name": "Sensor",
    "note": "Testing a sensor.",
    "serial": "A020000102"
  }
```

### Request 2

GET request to fetch a device information based on provided ID

```
HTTP Method: GET
URL: https://<api-gateway-url>/api/devices/{desired_id}
```

## Prerequisites

The package applies different tech stacks which needs to be installed before running the code. The prerequisites includes:

- Python 3.x
- GO programming language
- AWS CLI
- Go Dep (Dependency management tool)
- NodeJS
- Serverless
- Git

## Installation

Much of the installation part is organized based of the Windows platform. First of all you should install <a href="https://www.python.org/downloads/">Python 3.x</a> and <a href="https://git-scm.com/downloads">Git</a> if you don't have already.

### Install AWS CLI

If you already have installed pip, you can install the <a href="https://docs.aws.amazon.com/cli/latest/userguide/cli-chap-install.html">AWS CLI</a> (Command Line Interface) by using the following command:

```bash:
$ pip install awscli --upgrade --user
```

Then you can configure AWS CLI with your credentials and region information:

```bash:
$ aws configure
```

### Install Golang

Install <a href="https://golang.org/doc/install">Golang</a> and then set GOPATH and GOBIN in environment variables.

### Install Go Dep

```bash:
curl https://raw.githubusercontent.com/golang/dep/master/install.sh | sh
```

### Install Serverless

First, you need to install <a href="https://nodejs.org/en/download/">NodeJs</a> and then begin to install serverless using following command:

```bash:
npm install -g serverless
```

You can verify the installation using:

```bash:
serverless --version
```

## Project Directories

- <a href="https://github.com/mohammadmjn/aws-restful-api/tree/master/device">device</a> : This directory contains a package which implements the structure of items (device) we can put or fetch from our DynamDB table.

- <a href="https://github.com/mohammadmjn/aws-restful-api/tree/master/getDevice">getDevice</a> : This directory includes two files in order to implement GET request in real world tests. The `get.go` file implements the main functions for fetching information of a device from DynamoDB table if it exists in the table. The `get_test.go` includes integration test which uses to test the code in real world.

- <a href="https://github.com/mohammadmjn/aws-restful-api/tree/master/postDevice">postDevice</a> : It uses to implements POST request in real world tests. The `post.go` file includes the main functions for putting a device into DynamoDB table. The `post_test.go` includes integration test which uses to test POST request in real world.

- <a href="https://github.com/mohammadmjn/aws-restful-api/tree/master/unitTest">unitTest</a> : unitTest includes codes used to implement mock test of codes. It contains two subdirectories: `getDevice` & `postDevice`. These two subdirectories are like previous one with some differences in their `get_test.go` and `post_test.go` files in order to mock the output of GET and POST requests respectively.

- <a href="https://github.com/mohammadmjn/aws-restful-api/blob/master/Makefile">Makefile</a> : which is necessary to compile the project.

- <a href="https://github.com/mohammadmjn/aws-restful-api/blob/master/serverless.yml">serverless.yml</a> : The core component which contains all configurations to deploy this project on AWS.

## Build

You have to clone the repository to `%GOPATH%/src` directory since it imports `device` struct base on its relative path, otherwise the compiler will raise and error.

then build the project using the following command:

```bash
cd %GOPATH%/src/aws-restful-api
make build
```

:exclamation::exclamation: Note that you have to add the following command at the end of `~/.bashrc` of git bash in Windows or Unix-based systems to add `TABLE_Name` and its corresponding value as an environment variable.

```bash
vim ~/.bashrc
export TABLE_NAME="aws-challenge-devices"
```

Another option to do this task if you don't want to manipulate your ~/.bashrc is that each time you open a new terminal in Unix-based systems or git bash in Windows, you have to execute following commands in order to export `TABLE_NAME` as environment variables:

```bash
export TABLE_NAME=aws-challenge-devices
```

## Deploy

Deploy the project to AWS using following command:

```bash
sls deploy
```

This command will give two URLs as endpoints for each request in the output.

## Unit Test

In order to run unit tests for each request, you should navigate to its corresponding directory in `unitTest` folder and run the test command. Remember to export `TABLE_NAME` on opening a new terminal.

For unit test of POST request:

```bash
cd unitTest/postDevice
go test -v
```

For unit test of GET request:

```bash
cd unitTest/getDevice
go test -v
```

## Integration Test

In order to test the code in real world (integration test) on AWS DynamoDB database (given endpoints), run the test files located in `postDevice` or `getDeivce` of main project directory. Thus, you should execute following command in terminal or git bash:

For integration test of POST request:

```bash
cd postDevice/
go test -v
```

For integration test of GET request:

```bash
cd getDevice/
go test -v
```

## Author

Mohammad Mojrian - AWS Serverless Restful API

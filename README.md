# A Simple Golang AWS Restful API

This package, which is developed in Golang, implements two simple HTTP request (POST, GET) on Amazon Web Services (AWS) in order to put an item in DynamoDB table through POST request, as well as fetch information about and specific item from DynamoDB table (if exists) via GET request. The package applies AWS Lambda function to interact with AWS interface.

## API Request-Responce Cycle

The API accepts the following JSON requests and produces the corresponding HTTP responses:

### Request 1:

Request to insert a new device to database(DynamoDB).

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

#### Response 1 - Success:

Provided data inserted to database(DynamoDB) successfully.

```
HTTP-Statuscode: HTTP 201
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

#### Response 1 - Failure 1:

If any of the payload fields are missing, response will have a descriptive error message for client.

```
HTTP-Statuscode: HTTP 400
"Following fields are not provided: id, serial, ..."
```

#### Response 1 - Failure 2:

If any exceptional situation occurs on the server side.

```
HTTP-Statuscode: HTTP 500
"Internal Server's Error occurred."
```

### Request 2:

Get a device based on provided id.

```
HTTP Method: GET
URL: https://<api-gateway-url>/api/devices/{id}

Replace {id} with desire device id
```

#### Response 2 - Success:

The desire id exists on DynamoDB.

```
HTTP-Statuscode: HTTP 200
content-type: application/json
body:
  {
    "id": "/devices/id1",
    "deviceModel": "/devicemodels/id1",
    "name": "Sensor",
    "note": "Testing a sensor.",
    "serial": "A020000102"
  }
```

#### Response 2 - Failure 1:

```
HTTP-Statuscode: HTTP 404
"Desired device with provided id was not founded."
```

#### Response 2 - Failure 2:

If any exceptional situation occurs on the server side.

````
HTTP-Statuscode: HTTP 500
"Internal Server's Error occured."

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
````

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

- <a href="https://github.com/mohammadmjn/aws-restful-api/tree/master/unitTest">unitTest</a> : unitTest includes codes used to implement mock test of
  codes. It contains two subdirectories: `getDevice` & `postDevice`. These two subdirectories are like previous one with some differences in their `get_test.go` and `post_test.go` files in order to mock the output of GET and POST requests respectively.

- <a href="https://github.com/mohammadmjn/aws-restful-api/blob/master/Makefile">Makefile</a> : which is necessary to compile the project.

<a href="https://github.com/mohammadmjn/aws-restful-api/blob/master/serverless.yml">serverless.yml</a> : The core component which contains all configurations to deploy this project on AWS.

## Build

You have to clone the repository to `%GOPATH%\src` directory since it imports `device` struct base on its relative path, otherwise the compiler will raise and error.

then build the project using the following command:

```bash
cd %GOPATH%\src\aws-restful-api
make build
```

:exclamation: Note that

Launch `scenario.launch` which is in launch folder. This file calls `human_scenario.py` script which is located in scripts folder and it's the main file of Human Navigation scenario:

```bash
roslaunch human_nav_scenario scenario.launch
```

## Testing the Scenario

Given that Oculus Rift may not be available at test time, there are 2 python scripts to complete testing of Human Navigation scenario. First of all, Run the scenario. Then, start simulation in Windows side (Unity). To send 'Guidance_request' to robot, run `guidance_msg_pub.py` in a new terminal:

```bash
rosrun human_nav_scenario guidance_msg_pub.py
```

After running this script, the robot will receive 'Guidance_request' message and will give instructions to the test subject (human).

In order to test ability of the scenario to handle switching between sessions, you can use `giveup_publisher.py`. To run this script:

```bash
rosrun human_nav_scenario giveup_publisher.py
```

## Author

Mohammad Mojrian - AWS Restful API

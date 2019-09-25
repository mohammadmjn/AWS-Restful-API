package main

import (
	"aws-restful-api/device"
	"net/http"
	"reflect"
	"testing"

	"github.com/aws/aws-lambda-go/events"
	"github.com/aws/aws-sdk-go/aws"
	"github.com/aws/aws-sdk-go/service/dynamodb"
	"github.com/aws/aws-sdk-go/service/dynamodb/dynamodbiface"
)

type mockDynamoDBClient struct {
	dynamodbiface.DynamoDBAPI
}

func (self mockDynamoDBClient) PutItem(input *dynamodb.PutItemInput) (*dynamodb.PutItemOutput, error) {
	mockOutput := new(dynamodb.PutItemOutput)
	return mockOutput, nil
}

func TestPutDevice(t *testing.T) {
	type inputItem struct {
		device map[string]*dynamodb.AttributeValue
	}
	testcase := struct {
		name          string
		inputDevice   inputItem
		ExpectedError error
		// expectedOutput  *dynamodb.PutItemOutput
	}{
		name: "Check AWS correct JSON POST request",
		inputDevice: inputItem{
			device: map[string]*dynamodb.AttributeValue{
				"id":          &dynamodb.AttributeValue{S: aws.String("/devices/id1")},
				"deviceModel": &dynamodb.AttributeValue{S: aws.String("/devicemodels/id1")},
				"name":        &dynamodb.AttributeValue{S: aws.String("Sensor")},
				"note":        &dynamodb.AttributeValue{S: aws.String("Testing a sensor.")},
				"serial":      &dynamodb.AttributeValue{S: aws.String("A020000102")},
			},
		},
		ExpectedError: nil,
	}

	_, err := PutDevice(testcase.inputDevice.device, &mockDynamoDBClient{})

	if err != testcase.ExpectedError {
		t.Errorf("result error: \"%v\", expected error: \"%v\"", err, testcase.ExpectedError)
	}
}

func TestPostHandler(t *testing.T) {
	dynamoSvcPrevVal := dynamoDBSvc
	dynamoDBSvc = mockDynamoDBClient{}
	defer func() { dynamoDBSvc = dynamoSvcPrevVal }()

	type request struct {
		request events.APIGatewayProxyRequest
	}
	testcases := []struct {
		name             string
		request          request
		expectedResponse events.APIGatewayProxyResponse
	}{
		{
			name: "Wrong http method",
			request: request{
				events.APIGatewayProxyRequest{
					HTTPMethod: "POT",
				},
			},
			expectedResponse: events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: Invalid http method",
			},
		},
		{
			name: "Incorrect Content-Type",
			request: request{
				events.APIGatewayProxyRequest{
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/xml",
					},
				},
			},
			expectedResponse: events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: Invalid content type. It is not in a JSON format",
			},
		},
		{
			name: "Invalid request path",
			request: request{
				events.APIGatewayProxyRequest{
					Path:       "/api/d",
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/json",
					},
				},
			},
			expectedResponse: events.APIGatewayProxyResponse{
				StatusCode: http.StatusForbidden,
				Body:       "403 Forbidden: You don't have permission to access /api/d",
			},
		},
		{
			name: "Request with empty body",
			request: request{
				events.APIGatewayProxyRequest{
					Path:       "/api/devices",
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/json",
					},
					Body: "",
				},
			},
			expectedResponse: events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: Invalid JSON request. All fields are empty",
			},
		},
		{
			name: "Bad JSON format",
			request: request{
				events.APIGatewayProxyRequest{
					Path:       "/api/devices",
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/json",
					},
					Body: "{{:\"id\":}\"/devices/id1\":{},\"deviceModel\":\"/devicemodels/id1\",\"name\":\"Sensor\",\"note\":\"Testing a sensor.\",\"serial\":\"A020000102\"}",
				},
			},
			expectedResponse: events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: Invalid JSON format",
			},
		},
		{
			"Request with missed field - ID",
			request{
				events.APIGatewayProxyRequest{
					Path:       "/api/devices",
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/json",
					},
					Body: "{\"id\":\"\",\"deviceModel\":\"/devicemodels/id1\",\"name\":\"Sensor\",\"note\":\"Testing a sensor.\",\"serial\":\"A020000102\"}",
				},
			},
			events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: No value is specified for field(s) 'id' of device in json request",
			},
		},
		{
			"Request with multiple missed fields - Id, Name, Note",
			request{
				events.APIGatewayProxyRequest{
					Path:       "/api/devices",
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/json",
					},
					Body: "{\"id\":\"\",\"deviceModel\":\"/devicemodels/id1\",\"name\":\"\",\"note\":\"\",\"serial\":\"A020000102\"}",
				},
			},
			events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: No value is specified for field(s) 'id' 'name' 'note' of device in json request",
			},
		},
		{
			"Complete request without error",
			request{
				events.APIGatewayProxyRequest{
					Path:       "/api/devices",
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/json",
					},
					Body: "{\"id\":\"id1\",\"deviceModel\":\"/devicemodels/id1\",\"name\":\"Sensor\",\"note\":\"Testing a sensor.\",\"serial\":\"A020000102\"}",
				},
			},
			events.APIGatewayProxyResponse{
				StatusCode: http.StatusCreated,
				Body:       "{\"id\":\"id1\",\"deviceModel\":\"/devicemodels/id1\",\"name\":\"Sensor\",\"note\":\"Testing a sensor.\",\"serial\":\"A020000102\"}",
			},
		},
	}
	for _, testcase := range testcases {
		t.Run(testcase.name, func(t *testing.T) {
			if gotResponse, _ := PostHandler(testcase.request.request); !reflect.DeepEqual(gotResponse, testcase.expectedResponse) {
				t.Errorf("%s \nGot output from postHandler() = %v\nExpected output = %v", testcase.name, gotResponse, testcase.expectedResponse)
			}
		})
	}
}

func TestValidateDeviceInfo(t *testing.T) {
	testcases := []struct {
		name             string
		device           device.Device
		expectedResponse []string
	}{
		{
			"Missed field - ID",
			device.Device{
				Id:          "",
				DeviceModel: "/devicemodels/id1",
				Name:        "Sensor",
				Note:        "Testing a sensor.",
				Serial:      "A020000102",
			},
			[]string{"'id' "},
		},
		{
			"Missed field - DeviceModel",
			device.Device{
				Id:          "/devices/id1",
				DeviceModel: "",
				Name:        "Sensor",
				Note:        "Testing a sensor.",
				Serial:      "A020000102",
			},
			[]string{"'device model' "},
		},
		{
			"Missed field - Name",
			device.Device{
				Id:          "/devices/id1",
				DeviceModel: "/devicemodels/id1",
				Name:        "",
				Note:        "Testing a sensor.",
				Serial:      "A020000102",
			},
			[]string{"'name' "},
		},
		{
			"Request with missed field - Note",
			device.Device{
				Id:          "/devices/id1",
				DeviceModel: "/devicemodels/id1",
				Name:        "Sensor",
				Note:        "",
				Serial:      "A020000102",
			},
			[]string{"'note' "},
		},
		{
			"Request with missed field - Serial",
			device.Device{
				Id:          "/devices/id1",
				DeviceModel: "/devicemodels/id1",
				Name:        "Sensor",
				Note:        "Testing a sensor.",
				Serial:      "",
			},
			[]string{"'serial' "},
		},
		{
			"Multiple missed fields - Id, Name, Serial",
			device.Device{
				Id:          "",
				DeviceModel: "/devicemodels/id1",
				Name:        "",
				Note:        "Testing a sensor.",
				Serial:      "",
			},
			[]string{"'id' ", "'name' ", "'serial' "},
		},
	}
	for _, testcase := range testcases {
		t.Run(testcase.name, func(t *testing.T) {
			if gotResponse := ValidateDeviceInfo(testcase.device); !reflect.DeepEqual(gotResponse, testcase.expectedResponse) {
				t.Errorf("%s \nGot output from ValidateDeviceInfo() = %v\nExpected output = %v", testcase.name, gotResponse, testcase.expectedResponse)
			}
		})
	}
}

package main

import (
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

func (client mockDynamoDBClient) GetItem(input *dynamodb.GetItemInput) (*dynamodb.GetItemOutput, error) {
	mockOutput := new(dynamodb.GetItemOutput)
	id := input.Key["id"].S
	if *id == "id1" {
		mockOutput.SetItem(
			map[string]*dynamodb.AttributeValue{
				"id":          &dynamodb.AttributeValue{S: aws.String("id1")},
				"deviceModel": &dynamodb.AttributeValue{S: aws.String("/devicemodels/id1")},
				"name":        &dynamodb.AttributeValue{S: aws.String("Sensor")},
				"note":        &dynamodb.AttributeValue{S: aws.String("Testing a sensor.")},
				"serial":      &dynamodb.AttributeValue{S: aws.String("A020000102")},
			},
		)
	}
	return mockOutput, nil
}

func TestGetDevice(t *testing.T) {
	mockCorrectOutput := dynamodb.GetItemOutput{}
	mockCorrectOutput.SetItem(
		map[string]*dynamodb.AttributeValue{
			"id":          &dynamodb.AttributeValue{S: aws.String("id1")},
			"deviceModel": &dynamodb.AttributeValue{S: aws.String("/devicemodels/id1")},
			"name":        &dynamodb.AttributeValue{S: aws.String("Sensor")},
			"note":        &dynamodb.AttributeValue{S: aws.String("Testing a sensor.")},
			"serial":      &dynamodb.AttributeValue{S: aws.String("A020000102")},
		},
	)
	mockEmptyOutput := dynamodb.GetItemOutput{}

	testcases := []struct {
		name           string
		deviceId       string
		expectedOutput dynamodb.GetItemOutput
		expectedError  bool
	}{
		{
			name:           "Existed device ID",
			deviceId:       "id1",
			expectedOutput: mockCorrectOutput,
			expectedError:  false,
		},
		{
			name:           "Non-existing device ID",
			deviceId:       "badId",
			expectedOutput: mockEmptyOutput,
			expectedError:  false,
		},
	}

	for _, testcase := range testcases {
		t.Run(testcase.name, func(t *testing.T) {
			gotOutput, err := GetDevice(testcase.deviceId, &mockDynamoDBClient{})
			if (err != nil) != testcase.expectedError {
				t.Errorf("GetDevice() error = %v, ExpectedErr %v", err, testcase.expectedError)
				return
			}
			if len(gotOutput.GoString()) != len(testcase.expectedOutput.GoString()) {
				t.Errorf("%s \nGot output from GetDevice() = %v\nExpected output = %v", testcase.name, gotOutput.GoString(), testcase.expectedOutput.GoString())
			}
		})
	}
}

func TestGetHandler(t *testing.T) {
	dynamoSvcPrevVal := dynamoDBSvc
	dynamoDBSvc = mockDynamoDBClient{}
	defer func() { dynamoDBSvc = dynamoSvcPrevVal }()

	testcases := []struct {
		name           string
		request        events.APIGatewayProxyRequest
		expectedOutput events.APIGatewayProxyResponse
	}{
		{
			name: "Wrong HTTP method",
			request: events.APIGatewayProxyRequest{
				HTTPMethod: "POST",
			},
			expectedOutput: events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: Invalid http method",
			},
		},
		{
			name: "Invalid URL - Wrong spelling",
			request: events.APIGatewayProxyRequest{
				Path:       "/api/devi/id1",
				HTTPMethod: "GET",
			},
			expectedOutput: events.APIGatewayProxyResponse{
				StatusCode: http.StatusForbidden,
				Body:       "403 Forbidden Access: URL path is incorrect",
			},
		},
		{
			name: "Invalid URL - Contains less parts",
			request: events.APIGatewayProxyRequest{
				Path:       "/devices/id1",
				HTTPMethod: "GET",
			},
			expectedOutput: events.APIGatewayProxyResponse{
				StatusCode: http.StatusForbidden,
				Body:       "403 Forbidden Access: URL path is incorrect",
			},
		},
		{
			name: "Empty device's ID",
			request: events.APIGatewayProxyRequest{
				Path:       "/api/devices/",
				HTTPMethod: "GET",
			},
			expectedOutput: events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: ID of device is missed in path",
			},
		},
		{
			name: "Not exist item",
			request: events.APIGatewayProxyRequest{
				Path:       "/api/devices/not_exist",
				HTTPMethod: "GET",
			},
			expectedOutput: events.APIGatewayProxyResponse{
				StatusCode: http.StatusNotFound,
				Body:       "404 Not Found: Requested item does not exist in the database",
			},
		},
		{
			name: "complete request",
			request: events.APIGatewayProxyRequest{
				Path:       "/api/devices/id1",
				HTTPMethod: "GET",
			},
			expectedOutput: events.APIGatewayProxyResponse{
				StatusCode: http.StatusOK,
				Body:       "{\"id\":\"id1\",\"deviceModel\":\"/devicemodels/id1\",\"name\":\"Sensor\",\"note\":\"Testing a sensor.\",\"serial\":\"A020000102\"}",
			},
		},
	}

	for _, testcase := range testcases {
		t.Run(testcase.name, func(t *testing.T) {
			if gotOutput := GetHandler(testcase.request); !reflect.DeepEqual(gotOutput, testcase.expectedOutput) {
				t.Errorf("%s \nGot output from GetHandler() = %v\nExpected output = %v", testcase.name, gotOutput, testcase.expectedOutput)
			}
		})
	}
}

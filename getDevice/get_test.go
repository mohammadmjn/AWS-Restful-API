package main

import (
	"net/http"
	"reflect"
	"testing"

	"github.com/aws/aws-lambda-go/events"
)

func TestGetHandlerIntegration(t *testing.T) {
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
				Path:       "/api/devices/id2",
				HTTPMethod: "GET",
			},
			expectedOutput: events.APIGatewayProxyResponse{
				StatusCode: http.StatusOK,
				Body:       "{\"id\":\"id2\",\"deviceModel\":\"/devicemodels/id2\",\"name\":\"Sensor\",\"note\":\"Testing a sensor.\",\"serial\":\"A020000102\"}",
			},
		},
	}

	for _, testcase := range testcases {
		t.Run(testcase.name, func(t *testing.T) {
			if gotOutput, _ := GetHandler(testcase.request); !reflect.DeepEqual(gotOutput, testcase.expectedOutput) {
				t.Errorf("%s \nGot output from GetHandler() = %v\nExpected output = %v", testcase.name, gotOutput, testcase.expectedOutput)
			}
		})
	}
}
